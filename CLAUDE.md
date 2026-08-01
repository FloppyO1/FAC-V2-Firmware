# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for the **F.A.C. V2 (Floppy Ant Controller)** — a combat-robot (antweight) control board by Floppy Lab. Target MCU is an **STM32F072CBT6** (Cortex-M0, 48 MHz, no FPU → `-mfloat-abi=soft`). The board drives 3 DC motors, 2 HV servo outputs, reads a PWM/PPM RC receiver, and is configured over USB CDC by the external [FAC Tool](https://factool.floppylab.it/).

This directory is a **git submodule** of the `Floppy-Ant-Controller` repo (parent holds `/docs` manuals, `/HARDWARE V2`, `/TOOLS` python helpers + jingle composer, and the top-level `Readme.md`).

[docs/README_API.md](docs/README_API.md) is the full architecture + API reference for this firmware — read it before making non-trivial changes, and keep it in sync when you change a public function signature or the USB protocol.

**All code comments, documentation and READMEs in this project are written in English**, regardless of the language of the conversation.

## Build / flash / debug

STM32CubeIDE (Eclipse CDT) project — `.project`, `.cproject`, `FAC_Firmware_V2.ioc`. Two configurations exist: `Debug` and `Release`.

- **Normal workflow is the IDE**: build with STM32CubeIDE, flash/debug over ST-LINK using `FAC_Firmware_V2 Debug.launch`.
- **CLI build**: `make -C Debug all` (needs `arm-none-eabi-gcc` from *GNU Tools for STM32 13.3.rel1* and `make` on PATH — neither is on PATH in this environment by default).
- `Debug/makefile` and `Debug/**/subdir.mk` are **auto-generated and contain absolute paths** (`D:\GITHUB\Floppy-Ant-Controller\...`). Never hand-edit them; adding a new `.c` file requires regenerating them from the IDE (refresh/rebuild the project), otherwise the file silently isn't compiled.
- Compile flags: `-mcpu=cortex-m0 -mthumb -mfloat-abi=soft -std=gnu11 -O3 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -ffunction-sections -fdata-sections **-Wall** -fstack-usage -fcyclomatic-complexity --specs=nano.specs`. Linker script `STM32F072CBTX_FLASH.ld`. **`-Wall` is on**, so an unused `static` function or variable is a real warning in the build log — treat those as defects to clean up, not noise.
- **`Debug/` and `Release/` are git-ignored** (see `.gitignore`) — build artifacts and the generated makefiles are not versioned, so a fresh clone has to be built from the IDE once before `make -C Debug all` works. They were tracked up to and including commit `ac46e5f`, so `.o`/`.elf`/`.map` from older revisions are still reachable in history.
- There are **no unit tests** and no host-side test harness. Verification is on hardware (or via the FAC Tool with `IM_TESTING_FAC_TOOL`, see below).

### Regenerating from CubeMX

`FAC_Firmware_V2.ioc` regenerates `Core/Src/{main,gpio,tim,adc,dma,i2c,spi,usart,iwdg}.c`, `USB_DEVICE/`, `Drivers/`, `Middlewares/`. Edits to those files must live inside `/* USER CODE BEGIN X */ … /* USER CODE END X */` blocks or they will be lost. All application logic lives in `Core/Src/FAC_Code/` and is untouched by regeneration.

## Architecture

Bare-metal superloop, no RTOS. `main()` inits peripherals → `FAC_app_init()` → `FAC_app_main_loop()` forever (~13 ms/iteration ≈ 76 Hz with the tank mix + two direct links).

### Signal chain

```
RC receiver ──EXTI+TIM2──> fac_pwm_receiver / fac_ppm_receiver
                                  │  (raw µs pulse widths)
                                  ▼
                          fac_std_receiver          channels[] 0..999, deadzone applied
                                  │
                                  ▼
              fac_mixes  /  fac_functions           normalized floats [-1.0f, +1.0f]
                                  │
                                  ▼
                             fac_mapper             resolves link values → devices
                                  │
                       ┌──────────┴──────────┐
                       ▼                     ▼
                 fac_motors             fac_servo
              (DMA soft-PWM)          (TIM3 CH3/CH4)
```

- **`fac_std_receiver`** is the single abstraction all consumers use (`FAC_std_receiver_GET_channel(n)`, **1-based**). It lazily recalculates the channel from whichever receiver backend is active, applies the deadzone (center + extremes; channel 3 gets extremes only since throttle has no return spring), and clamps to `[0, RECEIVER_CHANNEL_RESOLUTION-1]`. Adding a receiver type = new `RECEIVER_TYPE_*` enum entry + `switch` cases in `FAC_std_reciever_init`, `FAC_std_receiver_GET_channel`, and `HAL_GPIO_EXTI_Callback`.
- **Timing capture**: TIM2 is a free-running 32-bit counter at **0.5 µs/tick** (prescaler 24-1 on 48 MHz). `MAX_TIM2_TEORETICAL_CHANNEL_COUNT 4000` = 2 ms. PWM mode uses 4 EXTI pins (CH1–CH4); PPM mode uses CH1 only, 8 channels.
- **Mixes vs. special functions**: a *mix* has up to 8 inputs / 10 outputs and only **one is active at a time** (`FAC_SETTINGS_CODE_ACTIVE_MIX`). A *special function* has exactly 1 input / 1 output, and up to 20 can run simultaneously; a function slot is disabled when its input channel setting is 0. Both write normalized `[-1.0f, +1.0f]`.
- **Mapper link encoding** (this is the crux of the configurability): each device setting (`FAC_SETTINGS_CODE_MAPPER_M1..S2`) stores `0` = unused, `100+i` = output *i* of the active mix, `200+i` = output of special function *i* (index into `FAC_SPECIAL_FUNCTIONS_ID`). `FAC_mapper_apply_to_devices()` runs each loop, updates only the mixes/functions actually referenced, then converts: motors get sign→direction + magnitude→speed; servos get `map_float(-1..1 → 0..SERVO_POSITION_RESOLUTION)`. **Only `100..109` and `200..210` are meaningful**, but the setting's `{min, max}` is a single `0..210` interval that cannot express the gap, so `110..199` is accepted by the validation layer — `FAC_mixes_GET_output()` / `FAC_functions_GET_output()` therefore bounds-check the index themselves and return `0.0f`. Keep that guard if you touch them.
- **Unmapped devices are forced safe**: motors to speed 0, servos PWM disabled.

### State machine (`FAC_app_main_loop`)

`FAC_STATE_DISARMED` → `FAC_STATE_NORMAL` → `FAC_STATE_CUTOFF`.

- Boot is always DISARMED. Arming additionally **requires the receiver to be seen connected** (`FAC_std_receiver_GET_is_connected()` — true once any channel reads non-zero). With `FAC_SETTINGS_CODE_ARMING_CHANNEL == 0` it arms immediately once connected; otherwise the channel must exceed `ARMING_THRESHOLD` (80%).
- CUTOFF is latched (no transition out) once the **cell** voltage stays at or below `CUTOFF_VOLTAGE_MV` for `CUTOFF_DETECTION_TIME` seconds. `LOW_BATTERY_VOLTAGE_MV` and `CUTOFF_VOLTAGE_MV` are both per-cell and are compared against `FAC_battery_GET_cell_voltage()`. Special cases are explicit: a `0` threshold (user-disabled) and `BATTERY_TYPE_USB` never cut off, while `BATTERY_TYPE_NONE` (unknown cell count → cells cannot be protected) always does.
- LED patterns and motor "beeps" per state are driven by the `*_LED_PERIOD` / `*_TONE_PERIOD` defines in `config.h`.

### Settings + EEPROM + USB protocol

`fac_settings.c` is the configuration hub. `enum FAC_SETTINGS_CODE` (in `fac_settings.h`) and the `settings[]` table (in `fac_settings.c`) are **positionally coupled** — the table is indexed by the enum value and must list rows in exactly the same order, each with `{code, default, min, max}`. `FAC_settings_SET_value()` rejects codes `>= FAC_SETTINGS_CODE_LAST` and clamps accepted values to `[min, max]`, so the table *is* the validation layer. The same bounds check guards `FAC_settings_GET_value()` and the two `FAC_settings_USB_SEND_setting_*()` helpers — the code byte arrives unvalidated from USB, so **any new function indexing `settings[]` must check it too**.

Adding a setting: append to the enum before `FAC_SETTINGS_CODE_LAST`, append the matching row, and consume it wherever needed (usually in a module's `init()`, since `FAC_app_init_all_modules()` re-runs all module inits when the tool sends *apply*). The FAC Tool addresses settings by these numeric codes, so **inserting in the middle breaks tool compatibility and shifts every EEPROM slot**.

- **EEPROM** (I2C1, `fac_eeprom.c`): each setting is a `uint16_t` at byte address `code * 2`; ~125 settings fit in the 2 kbit part. Writes are skipped when the value is unchanged (wear reduction) and each byte write costs a 10 ms blocking delay.
- **Factory-reset mechanism**: `FAC_settings_init(FIRMWARE_VERSION_TAG)` compares a boot marker byte in EEPROM with `FIRMWARE_VERSION_TAG` (a hash of MAJOR/MINOR/PATCH from `config.h`). **Bumping the firmware version in `config.h` therefore wipes user settings back to defaults on the next boot** — intended when the settings layout changes, but be aware it happens for any version bump. Boot blink count tells them apart: 10 fast blinks = defaults written, 3 blinks = normal load.
- **USB CDC protocol** (`FAC_USB_COMMAND_CODE`): host sends `[command, settingCode, valueMSB, valueLSB]`. Multi-byte values on the wire are **big-endian** while the MCU is little-endian — conversions are explicit in `FAC_settings_uint16_to_bytes` / `_bytes_to_uint16`, which are exact inverses (both MSB-first), so callers pass the wire bytes straight through without swapping. Most commands ack with a single byte `73`.
- Telemetry response is a **27-byte packet**: `[0]` code, `[1..16]` 8 channels, `[17..18]` Vbat mV, `[19]` battery type, `[20]` FAC state, `[21..26]` accel X/Y/Z in mg. The layout is documented above `FAC_settings_USB_SEND_telemetry()` — keep that comment in sync with the FAC Tool.
- `CDC_Receive_FS` (ISR context) only copies into `comSerialBuffer` and sets `newComSerialReceived`; the actual command handling runs from the main loop. Keep it that way — command handlers do blocking I2C/EEPROM work.

### Motor PWM (soft PWM via DMA)

`Core/Src/Libraries/DMApwm.c` generates PWM on **all 6 motor pins simultaneously** by DMA-ing a 1000-entry word buffer into `GPIOA->BSRR`, paced by TIM1 update events. Consequences:

- **Only port A pins work** (the port-B buffer is present but commented out). All motor pins are PA2–PA7.
- Frequency changes are done by writing `htim1.Instance->ARR = (TIMER_FREQ / (PWM_STEPS * freq)) - 1`, which is how `FAC_motor_make_noise()` turns the motors into a buzzer (blocking — it refreshes the IWDG in its loops). **The audible tone is that PWM repetition rate**, not the direction-toggling loop at the end of the function: the coils get a 5 % duty pulse train at `freq` and keep sounding for the whole wait loop (`HAL_Delay(0)` still costs ~1 ms per call because the HAL rounds up to the next tick). Don't "optimize away" that loop — it is what gives each note its length.
- Motor logic is **IN/IN driver, not PH/EN**. With brake enabled the duty is *inverted* (`MAX_DMA_PWM_VALUE - speed` on one pin, full on the other); with brake disabled it's plain duty + 0. See the truth table in `FAC_motor_apply_settings()`.

### Other modules

- **`fac_servo`**: TIM3 CH3/CH4, prescaler tuned for **1000 ticks per ms**, so `CCR = min_us_value + p` works at any frequency; period is recomputed as `(1000 * SERVO_RESOLUTION) / fs`. `p = (span * position) / MAX_SERVO_VALUE` with a **32-bit intermediate** — the widest span (2800 µs) times position 999 overflows 16 bit. Servos are disabled (CCR = 0) unless mapped and armed.
- **`fac_adc`**: 3 channels DMA-scanned (VBAT, AUX, VREFINT). VDDA is derived from the factory `VREFINT_CAL_ADDR` calibration value, which is what makes battery readings accurate across supply variation.
- **`fac_battery`**: `voltage_divider_ratio = 7692`; cell count is auto-detected once at boot from Vbat (USB ≈ 5.1 V is detected separately) and stored in `battery.type`. A user-settable `BATTERY_CALIBRATION` offset (stored as signed in a `uint16_t` slot) is added to readings. `enum BATTERY_TYPE` doubles as the cell count for `1S..4S` (values 1–4), which is what `FAC_battery_GET_cell_voltage()` divides by. **The numeric values are FAC Tool ABI** — they go out in telemetry byte `[19]`, so they must never be renumbered; guard the `USB` (0) and `NONE` (5) cases explicitly instead of doing arithmetic on them.
- **`fac_imu` / `Libraries/LSM6DS3`**: accel+gyro over I2C1. **Always check `FAC_IMU_GET_status() != HAL_ERROR` before trusting IMU data** — init failure is signalled by 20 LED blinks at boot and is non-fatal.
- **`jingles/`**: startup melodies played through the motors. The parent repo's `TOOLS/FAC_jingle_composer.html` generates these.

### Watchdog

IWDG is enabled with a ~500 ms timeout and refreshed once per main-loop pass. **Any loop or delay longer than ~500 ms must call `HAL_IWDG_Refresh(&hiwdg)` inside it** — existing code does this in EEPROM writes, IMU init, boot blink sequences, and `FAC_motor_make_noise`. `__HAL_DBGMCU_FREEZE_IWDG()` in `main()` keeps it from firing while halted in the debugger.

## Extending: mixes and special functions

Both have a **9-step numbered recipe in the file header comments** — follow them literally, they mark the exact places to edit:

- New mix: copy `Core/{Src,Inc}/FAC_Code/mixes_functions/mixes/fac_template_mix.{c,h}.template` → recipe in `fac_simple_tank_mix.c`.
- New special function: copy `…/functions/fac_template_function.{c,h}.template` → recipe in `fac_direct_link_function.c`.

In both cases the touch points are: the ID enum (`FAC_MIXES_ID` / `FAC_SPECIAL_FUNCTIONS_ID`), a `case` in the dispatcher (`FAC_mix_update()` / `FAC_functions_update()`), the `#include` in `fac_mixes.c` / `fac_functions.c`, and — for mixes — the `max` of `FAC_SETTINGS_CODE_ACTIVE_MIX` follows `FAC_MIX_LAST-1` automatically. The `/* INSERT YOUR CODE HERE */` region is the only part of the generated body to modify; the boilerplate around it (input fetch, output clamping, write-back) must stay.

Step 3's `mix_id` / `first_special_function_id` is a **documentation marker only** — it records which enum ID the file implements, and step 5's `case` label uses the enum name directly (a `static const` from another translation unit is neither visible nor a valid `case` label). It is therefore deliberately never read, and carries `__attribute__((unused))` so `-Wall` stays quiet; keep the attribute when copying the template.

`*.old` files are the superseded pre-refactor mix implementations, kept for reference; the dead code at the bottom of `fac_mixes.c` is likewise historical.

## Conventions

- Naming: `FAC_<module>_<action>()`; **uppercase `GET_`/`SET_`** marks accessors, and setters are usually `static` — modules expose behavior, not state. Module state lives in a single `static` struct instance per file.
- **Motors, servos, channels, and mix inputs are 1-based in public APIs**, arrays are 0-based — hence the pervasive `[n - 1]`. Mix/function *output indices*, by contrast, are 0-based.
- `TRUE`/`FALSE` (from `main.h`), not `stdbool`.
- Values crossing module boundaries are normalized floats `[-1.0f, +1.0f]`; `map_float`/`map_int32`/`map_uint32` in `fac_app.c` do the range conversions. Inner mix math often uses scaled integers (×1000) because M0 has no FPU.
- Comments and identifiers contain non-native-English spellings (`PROTORYPES`, `PUBBLIC`, `outouts`, `SPECIAL_FUNCITONS_NUMBER`, `FAC_std_reciever_init`). Match the existing spelling when referencing them; don't "fix" them casually — several are part of the public API surface.
- Tabs for indentation, K&R braces, doc blocks use `@brief`/`@note`/`@retval`/`@IMPORTANT`.

## Known issues

The list found during the API documentation pass is now **closed**: #1–#4, #6, #7 and #9–#18 are fixed, #5 was withdrawn as a misdiagnosis. Full descriptions, with the mechanism and the fix applied to each, in [README_API.md § 11](docs/README_API.md#11-known-issues). **Numbering is stable** — a closed issue keeps its number and moves to [§ 11.1 Fixed](docs/README_API.md#111-fixed) or [§ 11.2 Withdrawn](docs/README_API.md#112-withdrawn-not-bugs); never renumber the others.

Lesson worth keeping: that list was a set of *suspicions*, not verified defects. Issue #5 turned out to be a misreading of working code and "fixing" it would have broken the jingles, while #17 — the only one with a real safety consequence left — was found by reading the code around #8, not from the list. Confirm the mechanism, and ask the user (who has the hardware) before changing behaviour.

**Not a defect — unimplemented feature**
8. `FAC_SPECIAL_FUNCTION_DC_SERVO_1ST/2ND/3RD` are in the enum and mappable (`200+8`…`200+10`) but have no `case` in `FAC_functions_update()`, so they output a constant `0.0f`. The function itself was never written — this is a feature to implement, not a bug to fix.

Verified as *not* problems: the `settings[]` table and `enum FAC_SETTINGS_CODE` match exactly (64 entries, same order).

Nothing else is open: #8 above is the only outstanding item, and it is a feature to write rather than a defect.

## Debug switches (`Core/Inc/FAC_Code/config.h`, `DEBUG` builds only)

- `IM_TESTING_FAC_TOOL` — runs only MCU + EEPROM: skips the state machine and receiver init, and makes telemetry emit **simulated** channel/voltage/state/accel data. Use for FAC Tool development without a robot attached.
- `SERIAL_DEBUG` — enables serial debug prints.
- Firmware version constants live here too, with the bump policy (MAJOR = breaking, MINOR = backward-compatible feature, PATCH = fix) — and see the EEPROM factory-reset side effect noted above.
