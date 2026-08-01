# F.A.C. V2 Firmware — Architecture & API Reference

Firmware for the **Floppy Ant Controller V2**, a control board for combat robots (antweight class) designed by Floppy Lab. This document explains how the firmware works and documents every public function, so that someone picking up the project can be productive quickly.

> Companion documents: [CLAUDE.md](CLAUDE.md) (build instructions, conventions, known issues) and the user manuals in the parent repository's `/docs`.

---

## Table of contents

1. [Overview](#1-overview)
2. [Hardware target](#2-hardware-target)
3. [Boot sequence](#3-boot-sequence)
4. [Main loop and operating states](#4-main-loop-and-operating-states)
5. [The signal chain](#5-the-signal-chain)
6. [Configuration model](#6-configuration-model)
7. [Timing and the watchdog](#7-timing-and-the-watchdog)
8. [API reference](#8-api-reference)
9. [USB protocol reference](#9-usb-protocol-reference)
10. [Extending the firmware](#10-extending-the-firmware)
11. [Known issues](#11-known-issues)

---

## 1. Overview

The firmware is **bare metal** — no RTOS, no dynamic allocation. A single super-loop runs a state machine at roughly 76 Hz. Its job is to read an RC receiver, transform those channel values through a user-configurable processing chain, and drive 3 DC motor outputs and 2 servo outputs — while enforcing arming, low-battery and cut-off safety rules.

Everything a user can configure (mixing behaviour, channel assignment, reversal, PWM frequencies, voltage thresholds…) is stored as a table of 16-bit settings in an external EEPROM, editable over USB by the external **FAC Tool**. The firmware itself contains no robot-specific behaviour: it is a configurable engine.

The code is organised in layers, each in its own module under `Core/Src/FAC_Code/`:

| Layer | Modules | Responsibility |
|---|---|---|
| Application | `fac_app` | Boot, state machine, super-loop |
| Configuration | `fac_settings`, `fac_eeprom` | Setting table, persistence, USB command handling |
| Processing | `fac_mixes`, `fac_functions`, `fac_mapper` | Transform channels into device outputs |
| Device drivers | `fac_motors`, `fac_servo` | Apply outputs to hardware |
| Input | `fac_std_receiver`, `fac_pwm_receiver`, `fac_ppm_receiver` | Decode the RC link |
| Sensors | `fac_adc`, `fac_battery`, `fac_imu` | Voltage and motion sensing |
| Support | `Libraries/DMApwm`, `Libraries/LSM6DS3`, `jingles/` | Soft-PWM engine, IMU driver, startup melodies |

**Naming conventions.** All public symbols are prefixed `FAC_<module>_`. An **uppercase** `GET_` / `SET_` marks a plain accessor; setters are usually `static`, because modules expose behaviour rather than state. Each module keeps its state in a single `static` struct instance.

**Indexing.** Motors, servos, receiver channels and mix inputs are **1-based** in public APIs (`FAC_motor_set_speed_direction(1, …)` is motor M1), while the underlying arrays are 0-based — hence the pervasive `[n - 1]`. Mix and special-function **output** indices, by contrast, are 0-based.

---

## 2. Hardware target

**STM32F072CBT6** — Cortex-M0 @ 48 MHz, 128 KB flash, 16 KB RAM, no FPU (compiled `-mfloat-abi=soft`, so floating point is emulated in software; hot paths use scaled integers instead).

| Peripheral | Use |
|---|---|
| TIM1 + DMA → `GPIOA->BSRR` | Soft-PWM engine for the 6 motor pins |
| TIM2 | Free-running 32-bit counter, 0.5 µs/tick — RC pulse timing |
| TIM3 CH3/CH4 | Hardware PWM for SERVO1 / SERVO2 |
| ADC + DMA | VBAT, ADC_AUX, VREFINT (3-channel scan) |
| I2C1 | EEPROM (`0xA0`) and LSM6DS3 IMU (`0x6A`) |
| USB FS (CDC) | Configuration and telemetry link to the FAC Tool |
| IWDG | ~500 ms independent watchdog |
| EXTI | Receiver channel pins CH1–CH4 |

**Pin map** (from `Core/Inc/main.h`):

| Signal | Pin | Signal | Pin |
|---|---|---|---|
| LED | PC14 | M1_F / M1_B | PA2 / PA3 |
| DIGITAL_AUX1 / AUX2 | PF0 / PF1 | M2_F / M2_B | PA4 / PA5 |
| VBAT (analog) | PA0 | M3_F / M3_B | PA6 / PA7 |
| ADC_AUX (analog) | PA1 | SERVO1 / SERVO2 | PB0 / PB1 |
| CH1 / CH2 / CH3 | PB5 / PB4 / PB3 | CH4 | PA15 |
| NRF24L01_CE | PA8 | | |

> All six motor pins are on **port A** by necessity: the soft-PWM engine DMAs into `GPIOA->BSRR` only (see [`DMApwm`](#87-librariesdmapwm--soft-pwm-engine)).

---

## 3. Boot sequence

`main()` (CubeMX-generated) initialises the clocks and peripherals, then hands over to `FAC_app_init()`, which performs the following in a fixed order — the order matters, because later steps depend on settings loaded by earlier ones:

1. **~300 ms settling delay** (watchdog refreshed around it).
2. `FAC_adc_Init()` — ADC calibration, start the DMA scan, then measure VREFINT to derive the true VDDA. This is what makes battery readings accurate regardless of supply variation.
3. `FAC_battery_init()` — reset the battery struct, set the hardware divider ratio (7692, i.e. 7.692:1).
4. `FAC_settings_init(FIRMWARE_VERSION_TAG)` — the key step: compares a marker byte in EEPROM against the firmware version tag. If they differ (first boot ever, or a firmware version bump), the **default** settings are written to EEPROM and the LED blinks **10 times fast**; otherwise the stored settings are loaded and the LED blinks **3 times**. Watching the boot blink tells you which happened.
5. **~1000 ms delay**, then IMU init (`FAC_IMU_init`, accelerometer, gyroscope). On failure the LED blinks **20 times** and the firmware continues without IMU data; on success the gyro offsets are computed.
6. `FAC_app_init_all_modules()` — initialises every setting-dependent module (battery calibration, motors, receiver, servos, mixes, special functions). **This same function is re-run when the FAC Tool sends the *apply settings* command**, which is how configuration changes take effect without a reboot.
7. State is forced to `FAC_STATE_DISARMED`, the battery type (cell count) is detected once, and a startup jingle plays.

---

## 4. Main loop and operating states

`FAC_app_main_loop()` runs forever. One iteration takes about **13 ms** (≈76 Hz) with the tank mix and two direct-link functions active.

Each iteration: handle a pending USB command (if any), refresh the watchdog, then execute the current state.

```
                    receiver connected
                    AND arming channel high (or unused)
      ┌────────────────────────────────────────────┐
      │                                            ▼
┌───────────┐                                ┌──────────┐
│ DISARMED  │◀───── arming channel low ──────│  NORMAL  │
└───────────┘                                └──────────┘
   (boot)                                          │
                                    Vbat below cut-off
                                    for CUTOFF_DETECTION_TIME
                                                    │
                                                    ▼
                                             ┌──────────┐
                                             │  CUTOFF  │  (latched — no way out
                                             └──────────┘   except a power cycle)
```

**`FAC_STATE_DISARMED`** — every motor is set to speed 0 and every servo PWM is disabled, on every iteration. The board beeps periodically through the motors and blinks the LED at 1 Hz. Arming requires **two** conditions:

- `FAC_std_receiver_GET_is_connected()` is true. This becomes true once *any* channel reads non-zero, on the assumption that a receiver not bound to a transmitter emits nothing. This gate was added deliberately so the board cannot arm without a live RC link.
- Either `ARMING_CHANNEL` is `0` (feature disabled → arm immediately), or that channel exceeds `ARMING_THRESHOLD` (80 % of full scale).

**`FAC_STATE_NORMAL`** — the working state. Calls `FAC_mapper_apply_to_devices()` (the whole processing chain), re-checks the arming channel, evaluates low battery and cut-off, and holds the LED solid (or blinks it at 2 Hz when the battery is low).

**`FAC_STATE_CUTOFF`** — same shutdown as DISARMED, plus a faster blink and a distinctive beep. Entered when the **cell** voltage stays at or below `CUTOFF_VOLTAGE_MV` continuously for `CUTOFF_DETECTION_TIME` seconds. **Latched**: there is no transition out of this state. Three cases never trigger it: `CUTOFF_VOLTAGE_MV == 0` (user-disabled), USB power (no pack to protect), and — conversely — an unrecognised pack (`BATTERY_TYPE_NONE`) *always* ends in cut-off, deliberately, because its cell count is unknown and the cells cannot be protected.

Both `LOW_BATTERY_VOLTAGE_MV` and `CUTOFF_VOLTAGE_MV` are **per-cell** thresholds and are compared against `FAC_battery_GET_cell_voltage()`.

Timings and thresholds are all `#define`s in `Core/Inc/FAC_Code/config.h`.

---

## 5. The signal chain

This is the heart of the firmware and the part worth understanding first.

```
   RC receiver
        │  EXTI edges + TIM2 timestamps
        ▼
┌───────────────────────────┐
│ fac_pwm_receiver          │   raw pulse widths in 0.5 µs ticks
│ fac_ppm_receiver          │
└───────────────────────────┘
        │  FAC_std_receiver_new_channel_value()
        ▼
┌───────────────────────────┐
│ fac_std_receiver          │   channels[8], integers 0 … 999
│  · deadzone + relinearise │   (RECEIVER_CHANNEL_RESOLUTION = 1000)
└───────────────────────────┘
        │  FAC_std_receiver_GET_channel(n)   1-based
        ▼
┌─────────────────┬─────────────────────────┐
│ fac_mixes       │ fac_functions           │   normalised integers  -1000 … +1000
│ 8 in / 10 out   │ 20 × (1 in / 1 out)     │
│ one active mix  │ all enabled ones run    │
└─────────────────┴─────────────────────────┘
        │  FAC_mixes_GET_output(i) / FAC_functions_GET_output(i)   0-based
        ▼
┌───────────────────────────┐
│ fac_mapper                │   resolves each device's "link value"
└───────────────────────────┘
        │
   ┌────┴─────┐
   ▼          ▼
fac_motors  fac_servo
```

### 5.1 Receiver decoding

TIM2 free-runs as a 32-bit counter at **0.5 µs per tick** (prescaler 24−1 on 48 MHz). A 2 ms pulse is therefore 4000 ticks — `MAX_TIM2_TEORETICAL_CHANNEL_COUNT`.

- **PWM mode** — up to 4 channels. `HAL_GPIO_EXTI_Callback` fires on both edges of CH1–CH4; rising edges store `t1`, falling edges store `t2`; the pulse width is `t2 - t1`.
- **PPM mode** — up to 8 channels on CH1 only. Rising-edge-to-rising-edge intervals are measured; an interval longer than `PPM_SYNC_LENGTH` (8000 ticks) is the frame sync and resets the channel index.

In both cases readings more than 10 % above the theoretical maximum are discarded as glitches, and readings below half scale are clamped up (some transmitters over-range).

### 5.2 Normalisation and deadzone

`fac_std_receiver` is the **single abstraction every consumer uses**. It hides which physical protocol is active, lazily recalculating the requested channel on each `GET`. It applies `FAC_std_receiver_calculate_dead_zone()`, which implements both a **centre** deadzone and **extremes** deadzones, and then re-linearises the remaining travel so full stick range is preserved:

```
               __      ← extremes deadzone: snap to full
              /
             /
           --         ← centre deadzone: snap to zero
          /
       __/            ← extremes deadzone: snap to full
```

Channel 3 is special-cased: as the throttle stick usually has no return spring, it gets only the extremes deadzone, not the centre one.

### 5.3 Mixes vs. special functions

| | Mix | Special function |
|---|---|---|
| Inputs | up to 8 | exactly 1 |
| Outputs | up to 10 | exactly 1 |
| Concurrency | **exactly one active** (`ACTIVE_MIX` setting) | up to **20 simultaneously** |
| Disabled when | — | its input channel setting is `0` |
| Purpose | coupled logic (e.g. differential steering) | independent per-channel behaviour |

Both produce **normalised integers in `[-1000, +1000]`** (`fac_value_t`, defined with the math primitives in `fac_math.h`). That is the contract between the processing layer and the mapper: for a DC motor, sign is direction and magnitude is speed; for a servo, `-1000` is one end of travel and `+1000` the other.

The scale is not arbitrary: it equals `RECEIVER_CHANNEL_RESOLUTION`, `MOTOR_SPEED_RESOLUTION` and `SERVO_POSITION_RESOLUTION`, so both ends of the chain convert exactly. There are **no floats left in this chain** — M0 has no FPU, and every float operation was a library call.

The only implemented mix is **simple tank** (differential steering): it takes throttle and steering, computes `left = throttle + steering`, `right = throttle - steering`, adds a correction term so the range is preserved, and scales back into `[-1, +1]`. The only implemented special function is **direct link**, which passes its input straight through — used to drive a servo or motor directly from a stick.

### 5.4 The mapper and link values

This is the mechanism that makes the board configurable. Each of the five physical devices (M1, M2, M3, S1, S2) has one setting holding a **link value**:

| Link value | Meaning |
|---|---|
| `0` | device unused — motors forced to 0, servo PWM disabled |
| `100 + i` | output *i* of the **active mix** (`i` = 0…9) |
| `200 + i` | output of **special function** *i* (`i` = index into `FAC_SPECIAL_FUNCTIONS_ID`) |

`FAC_mapper_apply_to_devices()` runs once per loop and:

1. Reads the five link values.
2. Updates *only* the mix and special functions actually referenced (each at most once per loop, tracked by local flags) — so unused processing costs nothing.
3. Converts each normalised output to the device's units: motors get sign → direction and `FAC_math_abs(value)` → speed; servos get `FAC_math_to_range(value, 0, SERVO_POSITION_RESOLUTION)`. Both are exact, because the normalised scale equals the device resolutions.

**Default configuration** (from the settings defaults): M1 ← mix output 0 (left track), M2 ← mix output 1 (right track), S1 ← special function 0 (direct link on channel 3), M3 and S2 unused.

---

## 6. Configuration model

### 6.1 The settings table

`fac_settings.c` holds one flat array indexed by `enum FAC_SETTINGS_CODE`:

```c
typedef struct Setting {
    uint8_t  code;
    uint16_t value;
    uint16_t min_value;
    uint16_t max_value;
} Setting;
```

There are currently **64 settings**. The enum and the table are **positionally coupled** — the table is indexed by the enum value, so the rows must appear in exactly the same order as the enum entries. `FAC_settings_SET_value()` rejects any `code >= FAC_SETTINGS_CODE_LAST` and clamps every accepted write to `[min, max]`, which makes the table the single validation layer for the whole firmware.

Groups: motors (7) · servos (7) · battery (3) · receiver (3) · mixes (17) · special functions (20) · mapper (5) · firmware version (1) · battery calibration (1).

### 6.2 Persistence

Settings live in an external 2 kbit I²C EEPROM. A setting with code *n* occupies bytes `n*2` and `n*2+1`, so about 125 settings fit. Writes are skipped when the value is unchanged (flash wear) and each byte write costs a blocking 10 ms delay, so a full save is slow — that is why saving is an explicit USB command, not automatic.

A marker byte at address 255 holds `FIRMWARE_VERSION_TAG`, a hash of the version numbers in `config.h`:

```c
#define FIRMWARE_VERSION_TAG (uint8_t)((MAJOR*101U + MINOR*7U + PATCH*3U) % 255U)
```

When the tag in EEPROM does not match the running firmware, the settings are considered incompatible and **all defaults are rewritten**. Practical consequence: **bumping the firmware version resets the user's configuration.** That is intended when the settings layout changes, but it happens on *any* version bump.

### 6.3 Runtime flow

USB reception happens in interrupt context but does almost nothing: `CDC_Receive_FS()` copies the packet into `comSerialBuffer` and raises the `newComSerialReceived` flag. The main loop then calls `FAC_settings_command_response()`, which does the actual work — important, because command handlers perform blocking I²C EEPROM access.

---

## 7. Timing and the watchdog

The IWDG is configured with roughly a **500 ms** timeout and refreshed once per main-loop pass.

> **Rule: any loop or delay that can exceed ~500 ms must call `HAL_IWDG_Refresh(&hiwdg)` inside it**, or the MCU resets.

Existing code follows this in EEPROM writes, IMU initialisation, the boot blink sequences, `FAC_motor_make_noise()` and `FAC_jingles_delay()`. `__HAL_DBGMCU_FREEZE_IWDG()` in `main()` stops the watchdog while halted in a debugger.

Approximate cost of one `FAC_STATE_NORMAL` iteration: mapper ≈ 8 ms, arming check ≈ 0.8 ms, battery ≈ 0.2 ms, cut-off ≈ 8 µs, USB command ≈ 1 µs.

---

## 8. API reference

Only functions declared in headers are part of the public API. Functions still marked ⚠ have a documented caveat — see [Known issues](#11-known-issues).

### 8.1 `fac_app` — application core

```c
void     FAC_app_init(void);
void     FAC_app_init_all_modules(void);
void     FAC_app_main_loop(void);
uint8_t  FAC_app_GET_current_state(void);
uint8_t  FAC_app_GET_battery_type(void);
uint32_t map_uint32(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
int32_t  map_int32 (int32_t  x, int32_t  in_min, int32_t  in_max, int32_t  out_min, int32_t  out_max);
float    map_float (float    x, float    in_min, float    in_max, float    out_min, float    out_max);
```

| Function | Description |
|---|---|
| `FAC_app_init` | Full boot sequence (§3). Call once from `main()` after the CubeMX peripheral init. |
| `FAC_app_init_all_modules` | Re-initialises every setting-dependent module: battery calibration, motors, receiver, servos, mixes, special functions. **Does not touch the EEPROM.** Call after changing settings to apply them live. |
| `FAC_app_main_loop` | One iteration of the state machine. Call repeatedly and forever. |
| `FAC_app_GET_current_state` | Current state, one of `FAC_STATE_DISARMED` / `FAC_STATE_NORMAL` / `FAC_STATE_CUTOFF`. |
| `FAC_app_GET_battery_type` | Cell count detected **once at boot**: `BATTERY_TYPE_USB`(0), `_1S`(1) … `_4S`(4), `_NONE`(5). |
| `map_uint32` / `map_int32` | Integer range conversion. Clamps `x` to `in_max` (but **not** to `in_min`). Uses 64-bit intermediates to avoid overflow. |
| `map_float` | Float range conversion. Clamps `x` to **both** ends. **No longer used by the mix/function/mapper chain**, which is integer only — kept for code outside it. |

Global variable exported for the USB layer:

```c
extern uint8_t newComSerialReceived;   // set by CDC_Receive_FS, cleared by the main loop
```

### 8.2 `fac_settings` — configuration and USB commands

```c
void     FAC_settings_init(uint8_t bootValue);
uint8_t  FAC_settings_command_response(void);
uint16_t FAC_settings_GET_value(uint8_t code);
void     FAC_settings_USB_SEND_setting_value (uint8_t code);
void     FAC_settings_USB_SEND_setting_ranges(uint8_t code);
void     FAC_settings_SEND_what_received(void);
void     FAC_settings_SET_calibration_offset(uint16_t value);
```

| Function | Description |
|---|---|
| `FAC_settings_init` | Loads settings from EEPROM, or writes the defaults if `bootValue` does not match the marker byte. Pass `FIRMWARE_VERSION_TAG`. Blocking (EEPROM I/O + blink feedback). |
| `FAC_settings_command_response` | Decodes and executes the command currently in `comSerialBuffer`. Returns `TRUE` if the command code was recognised. Call from the main loop when `newComSerialReceived` is set. |
| `FAC_settings_GET_value` | Value of setting `code`. Returns `0` for an out-of-range code (bounds-checked). |
| `FAC_settings_USB_SEND_setting_value` | Transmits `[READ_VALUE, code, valueMSB, valueLSB]`. A `code >= FAC_SETTINGS_CODE_LAST` is ignored and gets **no reply at all**. |
| `FAC_settings_USB_SEND_setting_ranges` | Transmits `[READ_RANGE, code, minMSB, minLSB, maxMSB, maxLSB]`. Same out-of-range handling: no reply. |
| `FAC_settings_SEND_what_received` | Echoes the whole 64-byte receive buffer. Debug aid; currently unused. |
| `FAC_settings_SET_calibration_offset` | Writes the battery calibration offset into the settings table, clamped. Must be called *before* the first EEPROM init to take effect. Currently unused. |

Global buffer exported for the USB layer:

```c
extern uint8_t comSerialBuffer[64];    // last packet received over CDC
```

### 8.3 `fac_eeprom` — persistence

```c
void     FAC_eeprom_init(uint8_t bootValue);
void     FAC_eeprom_store_value(uint8_t position, const uint16_t value);
uint16_t FAC_eeprom_read_value(uint8_t position);
uint8_t  FAC_eeprom_is_first_time(void);
uint8_t  FAC_eeprom_GET_is_first_boot_value(void);
void     FAC_eeprom_WRITE_frist_boot_value_in_eeprom(void);
```

| Function | Description |
|---|---|
| `FAC_eeprom_init` | Stores the expected marker value in RAM. `0xFF` is remapped to `0xFE` because `0xFF` is the erased-EEPROM value and could not be distinguished. Does **not** touch the bus. |
| `FAC_eeprom_store_value` | Writes a 16-bit value at byte address `position*2`. **Reads first and skips the write if unchanged.** Blocking: ~10 ms per byte. |
| `FAC_eeprom_read_value` | Reads the 16-bit value at `position*2`. |
| `FAC_eeprom_is_first_time` | `TRUE` when the marker byte in EEPROM differs from the expected value — i.e. settings must be reset to defaults. |
| `FAC_eeprom_GET_is_first_boot_value` | The expected marker currently held in RAM. |
| `FAC_eeprom_WRITE_frist_boot_value_in_eeprom` | Commits the marker byte, "blessing" the stored configuration. *(Note the typo `frist` in the public name.)* |

Byte order note: this module stores **LSB first**, unlike the USB protocol which is MSB first.

### 8.4 `fac_std_receiver` — receiver abstraction

```c
void     FAC_std_reciever_init(uint8_t type);            // note: "reciever"
uint16_t FAC_std_receiver_GET_channel(uint8_t chNumber);
void     FAC_std_receiver_new_channel_value(uint8_t chNumber, uint16_t value);
uint8_t  FAC_std_receiver_GET_is_connected(void);
```

| Function | Description |
|---|---|
| `FAC_std_reciever_init` | Clears all channels and initialises the backend for `type` (`RECEIVER_TYPE_PWM` / `_PPM` / `_NRF24` / `_ELRS`). The last two are placeholders. *(Note the misspelling in the public name.)* |
| `FAC_std_receiver_GET_channel` | Value of channel `chNumber` (**1-based**), `0 … RECEIVER_CHANNEL_RESOLUTION-1`. Triggers a recalculation from the active backend. Channels beyond the backend's capability return the last stored value; a `chNumber` outside `1 … RECEIVER_CHANNELS_NUMBER` returns `0` without touching any backend. |
| `FAC_std_receiver_new_channel_value` | Backend → abstraction entry point: applies the deadzone, clamps, and stores. |
| `FAC_std_receiver_GET_is_connected` | `TRUE` once any channel has read non-zero. Polls one channel per call, round-robin over `1 … RECEIVER_CHANNELS_NUMBER`. Used as the arming gate. |

Also defined here (overrides the HAL weak symbol):

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);   // dispatches to the active receiver backend
```

### 8.5 `fac_pwm_receiver` / `fac_ppm_receiver` — protocol backends

```c
/* PWM — up to 4 channels, one pin each */
void FAC_pwm_receiver_init(void);
void FAC_pwm_receiver_Callback(uint8_t edge, uint16_t GPIO_Pin);
void FAC_pwm_receiver_calculate_channel_value(uint8_t chNumber);   // no bounds check of its own

/* PPM — up to 8 channels multiplexed on CH1 */
void FAC_ppm_receiver_init(void);
void FAC_ppm_receiver_Callback(uint8_t edge);
void FAC_ppm_receiver_calculate_channels_values(void);
```

| Function | Description |
|---|---|
| `*_init` | Starts TIM2 and zeroes the timestamp state. |
| `FAC_pwm_receiver_Callback` | Called from the EXTI dispatcher. `edge` is `RISING`/`FALLING`; stores the TIM2 timestamp for the matching channel. |
| `FAC_pwm_receiver_calculate_channel_value` | Converts one channel's `t2 - t1` into normalised units and pushes it to `fac_std_receiver`. Ignores implausible widths. |
| `FAC_ppm_receiver_Callback` | Rising edges only; measures inter-pulse intervals and detects the frame sync gap. |
| `FAC_ppm_receiver_calculate_channels_values` | Converts **all 8** channels at once and pushes them. |

### 8.6 `fac_mapper` — output routing

```c
void FAC_mapper_apply_to_devices(void);
```

Reads the five mapper settings, updates only the referenced mix / special functions, converts their normalised outputs into device units and applies them. **Must be called every loop iteration** to keep outputs alive. Devices with link value `0` are actively disabled (motors to 0, servo PWM off).

**Link value validity** — only `100 … 109` (mix outputs) and `200 … 210` (special function outputs) decode to a real output, but the setting's `{min, max}` is a single `0 … 210` interval and cannot express the gap, so `110 … 199` reaches the mapper. The output accessors bounds-check the index themselves and return `FAC_VALUE_ZERO`, which makes an invalid link behave like "no output" instead of reading past the arrays — see issue #17.

### 8.7 `fac_mixes` — mix framework

```c
void  FAC_mixes_init(void);
void  FAC_mix_update(void);
void  FAC_mixes_update_mix_inputs(void);
void  FAC_mixes_update_mix_outputs(float mix_output[]);
float FAC_mixes_GET_input (uint8_t inputNumber);          // 0-based
float FAC_mixes_GET_output(uint8_t outputNumber);         // 0-based
```

| Function | Description |
|---|---|
| `FAC_mixes_init` | Loads the active mix ID and per-input channel/reversal settings; zeroes inputs and outputs. Requires settings to be loaded first. |
| `FAC_mix_update` | Dispatches to the active mix's update function. Add a `case` here for each new mix. |
| `FAC_mixes_update_mix_inputs` | Refreshes all 8 inputs from the receiver, normalising `0…999` → `-1000…+1000` with `FAC_math_from_range` and applying per-input reversal from the values cached by `init()`. Disabled inputs (channel `0`) become `FAC_VALUE_ZERO`. Called by the mix boilerplate. |
| `FAC_mixes_update_mix_outputs` | Copies a mix's local output array into the shared struct. **Must be called at the end of every mix update.** |
| `FAC_mixes_GET_input` / `_GET_output` | Read the shared normalised arrays. Both bounds-check the index and return `FAC_VALUE_ZERO` when it is out of range — the mapper can reach them with an invalid link value (issue #17). |

Both the input channel and the reversal flag are **cached at `init()`**, so changing either through USB takes effect on the next *apply*, not on the write itself.

### 8.8 `fac_functions` — special-function framework

```c
void  FAC_functions_init(void);
void  FAC_functions_update(uint8_t sFunctionID);
void  FAC_functions_update_input (uint8_t functionNumber);
void  FAC_functions_update_inputs(void);
void  FAC_functions_SET_output(uint8_t functionNumber, float outputValue);
float FAC_functions_GET_input (uint8_t functionNumber);
float FAC_functions_GET_output(uint8_t functionNumber);
```

| Function | Description |
|---|---|
| `FAC_functions_init` | Loads each function's input channel from settings; zeroes inputs and outputs. |
| `FAC_functions_update` | Dispatches to the implementation for `sFunctionID` (an `FAC_SPECIAL_FUNCTIONS_ID` value). The three `DC_SERVO` IDs are declared but their function was never written — see issue #8. |
| `FAC_functions_update_input` | Refreshes **one** slot from the receiver, normalised to `[-1000, +1000]`; a disabled slot (input channel `0`) is reset to `FAC_VALUE_ZERO`, and an out-of-range index is ignored. This is what the function boilerplate calls: a special function has exactly one input, so refreshing all 20 slots was the same work repeated once per linked function. |
| `FAC_functions_update_inputs` | Loops `FAC_functions_update_input` over all 20 slots. **Currently has no caller** — the boilerplate no longer uses it. Kept on purpose for a future function that has to read slots other than its own; call it before reading them. |
| `FAC_functions_SET_output` | Where a function publishes its result. Ignores an out-of-range `functionNumber`. |
| `FAC_functions_GET_input` / `_GET_output` | Read the shared arrays. Both bounds-check the index and return `FAC_VALUE_ZERO` when it is out of range — the mapper can reach them with an invalid link value (issue #17). |

Implemented behaviours:

```c
void FAC_simple_tank_mix_update(void);                      // differential steering
void FAC_direct_link_function_update(uint8_t sFunctionID);  // pass-through
```

### 8.9 `fac_motors` — DC motor outputs

```c
void FAC_motor_init(void);
void FAC_motor_set_speed_direction(uint8_t motorNumber, uint8_t dir, uint16_t speed);
void FAC_motor_set_brake_status(uint8_t motorNumber, uint8_t state);
void FAC_motor_enable_brake (uint8_t motorNumber);
void FAC_motor_disable_brake(uint8_t motorNumber);
void FAC_motor_is_reversed  (uint8_t motorNumber, uint8_t isReversed);
void FAC_motor_make_noise(uint16_t freq, uint16_t duration);   // blocking
```

| Function | Description |
|---|---|
| `FAC_motor_init` | Starts the soft-PWM engine at the configured frequency, binds pins, applies reversal/brake settings, forces all motors to speed 0. |
| `FAC_motor_set_speed_direction` | Sets and **immediately applies** direction (`FORWARD`/`BACKWARD`) and speed (`0 … MOTOR_SPEED_RESOLUTION-1`). `motorNumber` is 1–3. Reversal is applied internally. |
| `FAC_motor_set_brake_status` | Brake (`TRUE`) vs. coast (`FALSE`) mode. Takes effect on the next apply. |
| `FAC_motor_enable_brake` / `_disable_brake` | Convenience wrappers over the above. |
| `FAC_motor_is_reversed` | Sets the per-motor reversal flag. |
| `FAC_motor_make_noise` | Turns all three motors into a buzzer at `freq` Hz for `duration` ms, then restores the configured PWM frequency and stops the motors. **Blocking**; refreshes the watchdog internally. |

**How the buzzer actually works** — this is easy to misread. The tone is the **soft-PWM repetition rate itself**: `FAC_DMA_pwm_change_freq(freq)` sets `TIM1->ARR = TIMER_FREQ / (PWM_STEPS * freq) - 1`, so with `TIMER_FREQ = 48 MHz` and `PWM_STEPS = 1000` a `NOTE_C6 = 1047` gives `ARR = 44` → a PWM period of `1000 × 45 / 48 MHz = 0.9375 ms` ≈ 1067 Hz. The DMA keeps pushing that pulse train into the coils at 5 % duty until the frequency and speed are restored, so the note is heard **during the wait loop**, which runs ~1 ms per iteration because `HAL_Delay(0)` still waits for the next SysTick tick (`wait += uwTickFreq` in `stm32f0xx_hal.c`). The direction-toggling `while` below it only gets the leftover (≤ ~1 ms of a 125 ms note) and contributes essentially nothing. **Do not remove the wait loop** — it is what gives the note its length.

**Driver logic** — the H-bridges are driven **IN/IN**, not PH/EN, and the duty is *inverted* in brake mode:

| Mode | Forward | Reverse |
|---|---|---|
| Brake enabled | `pinB = MAX - speed`, `pinF = MAX` | `pinF = MAX - speed`, `pinB = MAX` |
| Coast enabled | `pinF = speed`, `pinB = 0` | `pinB = speed`, `pinF = 0` |

`FAC_motor_GET_direction`, `_GET_speed`, `_GET_reverse` and `_GET_brake_en` are declared in the header alongside the setters.

### 8.10 `fac_servo` — servo outputs

```c
void     FAC_servo_init(void);
void     FAC_servo_set_position(uint8_t servoNumber, uint16_t position);
void     FAC_servo_enable (uint8_t servoNumber);
void     FAC_servo_disable(uint8_t servoNumber);
void     FAC_servo_is_reversed(uint8_t servoNumber, uint8_t isReversed);
uint16_t FAC_servo_GET_position  (uint8_t servoNumber);
uint8_t  FAC_servo_GET_is_enable (uint8_t servoNumber);
uint8_t  FAC_servo_GET_is_reversed(uint8_t servoNumber);
```

| Function | Description |
|---|---|
| `FAC_servo_init` | Starts TIM3 CH3/CH4, applies frequency and per-servo min/max pulse settings, then **disables both servos** as a safety precaution. |
| `FAC_servo_set_position` | Sets and applies position `0 … MAX_SERVO_VALUE`, mapped into the configured `[min_us, max_us]` pulse window. Reversal applied internally. |
| `FAC_servo_enable` / `_disable` | Enables/disables the PWM. Disabling writes `CCR = 0`, producing no pulse at all. |
| `FAC_servo_is_reversed` | Sets the reversal flag and re-applies. |
| `FAC_servo_GET_*` | Accessors, including `FAC_servo_GET_servo_freq()`. |

TIM3 is prescaled to **1000 ticks per millisecond**, so a pulse width in microseconds maps directly onto the CCR value at any frame rate; the period is recomputed as `(1000 × SERVO_RESOLUTION) / frequency`.

Position → pulse width is `CCR = min_us + (span × position) / MAX_SERVO_VALUE`, where `span = max_us - min_us`. The multiplication is done in **32 bit on purpose**: the widest configurable span (2800 µs) times the maximum position (999) is ≈ 2.8 M and does not fit in 16 bit. Since `position` is clamped to `MAX_SERVO_VALUE`, full deflection reaches exactly the configured `max_us`. Note that `MAX_SERVO_VALUE` is parenthesised in the header precisely because it is used as a divisor here.


### 8.11 `fac_adc` — analog front end

```c
HAL_StatusTypeDef FAC_adc_Init(void);
uint16_t FAC_adc_GET_resolution(void);
uint32_t FAC_adc_GET_Vref_in_uV(void);
uint16_t FAC_adc_get_raw_channel_value(uint8_t chNumber);
```

| Function | Description |
|---|---|
| `FAC_adc_Init` | Calibrates the ADC, starts a 3-channel DMA scan, then measures VREFINT (20 samples) against the factory calibration at `0x1FFFF7BA` to derive the true VDDA. Blocking (~250 ms). |
| `FAC_adc_GET_resolution` | Full-scale count, e.g. 4096 for 12-bit. |
| `FAC_adc_GET_Vref_in_uV` | Measured VDDA in microvolts — the basis of accurate voltage readings. |
| `FAC_adc_get_raw_channel_value` | Raw DMA sample. Index 0 = VBAT, 1 = ADC_AUX, 2 = VREFINT. ⚠ No bounds check. |

### 8.12 `fac_battery` — battery monitoring

```c
void     FAC_battery_init(void);
uint16_t FAC_battery_GET_voltage(void);
uint16_t FAC_battery_GET_cell_voltage(void);
uint8_t  FAC_battery_GET_type(uint16_t vbat);
void     FAC_battery_calculate_type(uint16_t vbat);
void     FAC_battery_SET_calibration_offset(int16_t offset);
int16_t  FAC_battery_GET_calibration_offset(void);
```

| Function | Description |
|---|---|
| `FAC_battery_init` | Zeroes the struct and sets the hardware divider ratio (7692 = 7.692:1 ×1000). |
| `FAC_battery_GET_voltage` | Pack voltage in **millivolts** (8.02 V → 8020), averaged over 5 samples, calibration offset applied. |
| `FAC_battery_GET_cell_voltage` | Voltage of a **single cell** in millivolts — pack voltage divided by the cell count detected at boot. On USB power (`BATTERY_TYPE_USB`) or on an unrecognised pack (`BATTERY_TYPE_NONE`) there is no valid cell count, so the divider falls back to 1 and the pack voltage is returned. |
| `FAC_battery_GET_type` | Recomputes and returns the cell count for the given pack voltage. |
| `FAC_battery_calculate_type` | Classifies `vbat` as USB (≈5.1 V ±tolerance, accounting for the diode drop) or as an *n*-cell pack, using 3.8 V nominal ±425 mV per cell. Unrecognised → `BATTERY_TYPE_NONE`. |
| `FAC_battery_SET_calibration_offset` / `_GET_` | Signed millivolt correction applied to every reading. |

Conversion: `mV = (VDDA_µV / adcResolution) × rawSample × dividerRatio / 1e6`.

### 8.13 `fac_imu` — motion sensing

```c
HAL_StatusTypeDef FAC_IMU_init(void);
void FAC_IMU_init_accelerometer(void);
void FAC_IMU_init_gyroscope(void);
void FAC_IMU_compute_gyro_offset(void);
HAL_StatusTypeDef FAC_IMU_GET_status(void);
float FAC_IMU_GET_accel_X(void);  // g
float FAC_IMU_GET_accel_Y(void);
float FAC_IMU_GET_accel_Z(void);
float FAC_IMU_GET_gyro_X(void);   // deg/s
float FAC_IMU_GET_gyro_Y(void);
float FAC_IMU_GET_gyro_Z(void);
```

> **Always check `FAC_IMU_GET_status() != HAL_ERROR` before using IMU data.** An init failure is non-fatal — the firmware keeps running without the IMU — so consumers must handle it.

Each `GET_accel_*` / `GET_gyro_*` call performs a **blocking I²C read** of that axis; they are not cached. Accelerometer range is ±16 g at 416 Hz; gyroscope ±2000 dps at 416 Hz.

### 8.14 `Libraries/LSM6DS3` — IMU driver

```c
HAL_StatusTypeDef LSM6DS3_init(LSM6DS3 *obj, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LSM6DS3_init_accel(LSM6DS3 *obj);
HAL_StatusTypeDef LSM6DS3_init_gyro (LSM6DS3 *obj);
void LSM6DS3_update_accelerometer_single_value(LSM6DS3 *obj, uint8_t axis);
void LSM6DS3_update_accelerometer_all_values  (LSM6DS3 *obj);
void LSM6DS3_update_gyroscope_single_value    (LSM6DS3 *obj, uint8_t axis);
void LSM6DS3_update_gyroscope_all_values      (LSM6DS3 *obj);
void LSM6DS3_calculate_offset(LSM6DS3 *obj);
```

`LSM6DS3_init` verifies `WHO_AM_I == 0x6A` and returns `HAL_ERROR` if a different device answers. Results are written into the object's `acc_*` (g) and `gyro_*` (deg/s) fields; `axis` is `X_AXIS` / `Y_AXIS` / `Z_AXIS`.

`LSM6DS3_calculate_offset` averages 200 gyro samples per axis and stores the negated result in `gyro_offsets[LSM6DS3_AXIS_NUMBER]`, which `LSM6DS3_update_gyroscope_single_value` then adds to every reading. The call blocks for ~1 s and refreshes the IWDG in its loop.

### 8.15 `Libraries/DMApwm` — soft-PWM engine

```c
void    initDMApwm(uint16_t freq);
uint8_t setDMApwmDuty(GPIO_TypeDef *port, uint16_t pin, uint16_t duty);
void    FAC_DMA_pwm_change_freq(uint16_t freq);
```

Generates PWM on **all six motor pins simultaneously** by DMA-ing a 1000-word bit-pattern buffer into `GPIOA->BSRR`, paced by TIM1 update events. No timer channels are consumed, so any GPIO can be a PWM pin — but:

> **Only port A is supported.** The port-B buffer exists in the source but is commented out; `setDMApwmDuty` returns `0` for any other port.

| Function | Description |
|---|---|
| `initDMApwm` | Sets the timer period for `freq`, starts TIM1 and the DMA stream, zeroes the buffer. |
| `setDMApwmDuty` | Rewrites the bit pattern for one pin. `duty` is `0 … PWM_STEPS-1`. Returns `1` on success, `0` if the port is unsupported. |
| `FAC_DMA_pwm_change_freq` | Changes the frequency on the fly by rewriting `ARR` — this is how tone generation works. |

Frequency relation: `ARR = TIMER_FREQ / (PWM_STEPS × freq) - 1`, with `TIMER_FREQ` = 48 MHz and `PWM_STEPS` = 1000.

### 8.16 `jingles` — startup melodies

```c
void FAC_jingle_simple_scale(void);
void FAC_jingle_Tequila(void);
void FAC_jingle_Tequila_long(void);
void FAC_jingle_neverGiveYouUp(void);
```

Melodies played through the motor coils via `FAC_motor_make_noise`. All are **blocking** and only suitable at boot. Note frequencies come from `jingles/notes.h`; new melodies can be generated with `TOOLS/FAC_jingle_composer.html` in the parent repository.

### 8.17 USB CDC

```c
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
```

Returns `USBD_BUSY` if the previous transfer has not completed — **the firmware does not retry**, so a response can be silently dropped if the host polls faster than the device drains.

---

## 9. USB protocol reference

Virtual COM port (CDC). The host sends a small command packet; the device answers. **Multi-byte values are big-endian on the wire** (MSB first), while the MCU is little-endian — conversions are explicit in `FAC_settings_uint16_to_bytes()` and `FAC_settings_bytes_to_uint16()`, which are exact inverses, so callers hand the wire bytes over in order without swapping.

### Command codes (`enum FAC_USB_COMMAND_CODE`)

| Code | Name | Host sends | Device replies |
|---|---|---|---|
| 0 | `READ_VALUE` | `[0, code]` | `[0, code, valMSB, valLSB]` |
| 1 | `READ_RANGE` | `[1, code]` | `[1, code, minMSB, minLSB, maxMSB, maxLSB]` |
| 2 | `WRITE` | `[2, code, valMSB, valLSB]` | `73` (ack) |
| 3 | `PING` | `[3]` | `73` (ack) |
| 4 | `TELEMETRY_REQUEST` | `[4]` | 27-byte telemetry packet |
| 5 | `TELEMETRY_RESPONSE` | — | (marker byte of the telemetry packet) |
| 6 | `SAVE_TO_EEPROM` | `[6]` | `73` (ack) |
| 7 | `APPLY_SETTINGS` | `[7]` | `73` (ack) |

`WRITE` only updates RAM. `SAVE_TO_EEPROM` persists; `APPLY_SETTINGS` re-runs `FAC_app_init_all_modules()` so changes take effect without a reboot. The ack byte is `73` (`0x49`).

### Telemetry packet (27 bytes)

| Offset | Size | Content |
|---|---|---|
| 0 | 1 | `FAC_USB_COMMAND_TELEMETRY_RESPONSE` (5) |
| 1–16 | 16 | Channels 1–8, 2 bytes each, MSB first |
| 17–18 | 2 | Pack voltage in mV |
| 19 | 1 | Battery type: 0 = USB, 1–4 = cells, 5 = unknown |
| 20 | 1 | FAC state: 0 = DISARMED, 1 = NORMAL, 2 = CUTOFF |
| 21–22 | 2 | Accel X in mg (**signed**) |
| 23–24 | 2 | Accel Y in mg (**signed**) |
| 25–26 | 2 | Accel Z in mg (**signed**) |

> Keep this table, the comment above `FAC_settings_USB_SEND_telemetry()`, and the FAC Tool in sync.

With `IM_TESTING_FAC_TOOL` defined, all telemetry fields are **simulated** (channels ramp, voltage cycles through 1S–4S values, state cycles) so the tool can be developed without a robot.

---

## 10. Extending the firmware

### Adding a mix

Templates: `Core/{Src,Inc}/FAC_Code/mixes_functions/mixes/fac_template_mix.{c,h}.template`. A **9-step recipe** is in the header comment of `fac_simple_tank_mix.c` — follow it literally; the numbered markers show exactly where to edit. Summary:

1. Copy and rename both template files; update the include guards.
2. Add an ID to `enum FAC_MIXES_ID` in `fac_mixes.h` (before `FAC_MIX_LAST`).
3. Add a `case` in `FAC_mix_update()` and the `#include` in `fac_mixes.c`.
4. Document your inputs/outputs with `#define INPUT_*` / `OUTPUT_*` names.
5. Write your logic between the `/* INSERT YOUR CODE HERE */` markers only — the surrounding boilerplate (input fetch, clamping, write-back) must stay.

The `max` of `FAC_SETTINGS_CODE_ACTIVE_MIX` tracks `FAC_MIX_LAST-1` automatically.

### Adding a special function

Same idea; templates under `.../functions/`, recipe in the header of `fac_direct_link_function.c`. If the function is meant to be usable several times, add several consecutive IDs with `1ST`/`2ND`/… suffixes and group their `case` labels without `break` except on the last.

### Adding a setting

Append to `enum FAC_SETTINGS_CODE` **before** `FAC_SETTINGS_CODE_LAST`, append the matching `{code, default, min, max}` row in the same position in `settings[]`, and consume it in the relevant module's `init()` (so that *apply settings* picks it up).

> **Never insert a code in the middle.** The FAC Tool addresses settings by numeric code and the EEPROM address is `code*2`, so inserting shifts every later setting and silently corrupts stored configurations.

### Adding a receiver protocol

Add a `RECEIVER_TYPE_*` entry, then handle it in the three `switch` statements: `FAC_std_reciever_init()`, `FAC_std_receiver_GET_channel()` and `HAL_GPIO_EXTI_Callback()`. Your backend pushes decoded values through `FAC_std_receiver_new_channel_value()`. `NRF24` and `ELRS` are reserved placeholders — the hardware supports ELRS but it is not implemented.

### Build note

Adding a new `.c` file requires regenerating the STM32CubeIDE build files (`Debug/**/subdir.mk`), otherwise it is silently not compiled. See [CLAUDE.md](CLAUDE.md).

---

## 11. Known issues

Found while documenting the code. Numbering is kept stable, so entries move to [11.1 Fixed](#111-fixed) or [11.2 Withdrawn](#112-withdrawn-not-bugs) instead of being renumbered. The same list is tracked in [CLAUDE.md](CLAUDE.md).

The original list is now closed. What remains is one enum entry whose feature was never written:

| # | Severity | Location | Issue |
|---|---|---|---|
| 8 | **Not a defect** | `fac_functions.c` | `FAC_SPECIAL_FUNCTION_DC_SERVO_1ST/2ND/3RD` are declared in the enum and reachable via the mapper (`200+8` … `200+10`), but have no `case` in `FAC_functions_update()`, so they output a constant `0.0f`. The DC-servo function was simply never implemented — this is a feature to write, not a bug to fix. The read stays in bounds (the outputs array holds 20 entries). |

### 11.1 Fixed

| # | Severity | Location | Issue | Fix |
|---|---|---|---|---|
| 1 | **High** | `fac_std_receiver.c` `FAC_std_receiver_is_connected()` | The round-robin index reached `0` once every 9 calls, because the `if (channelToCheck == 0) channelToCheck++` guard corrected the *previous* value while the check used the *new* one. `FAC_std_receiver_GET_channel(0)` then read `channels[-1]`, and in PWM mode also called `FAC_pwm_receiver_calculate_channel_value(0)` — which reads `channels_t2[-1]`, i.e. `channels_t1[3]`, and on a passing range check writes `receiver.channels[-1]`, `pwmReceiver.channels_t1[-1]` and zeroes CH4's rising-edge timestamp. Garbage in the adjacent word could make the board report "receiver connected" and arm without an RC link. | The index now cycles with `channelToCheck = (channelToCheck % RECEIVER_CHANNELS_NUMBER) + 1`, which can only produce `1 … RECEIVER_CHANNELS_NUMBER`. `FAC_std_receiver_GET_channel()` additionally returns `0` for any `chNumber` outside that range, before touching any backend. |
| 2 | **High** | `fac_battery.c` `FAC_battery_GET_cell_voltage()` | Returned `battery.voltage` (pack) instead of `single_cell_voltage`, and `FAC_battery_calculate_cell_voltage()` never divided by the cell count — so `single_cell_voltage` was write-only dead state. Low-battery detection compared the pack voltage against a per-cell threshold (clamped to 2800…4000 mV), so on 2S and above **low battery could never trigger**: a 2S at 3.4 V/cell reads 6800 mV, well above a 3400 mV threshold. | `FAC_battery_calculate_cell_voltage()` now divides the pack voltage by the cell count held in `battery.type`, falling back to a divider of 1 when the type is `BATTERY_TYPE_USB` (value 0 — would be a division by zero) or `BATTERY_TYPE_NONE`. The getter returns `battery.single_cell_voltage`. `fac_app.c` renamed its local to `vcell` and now feeds both the low-battery and the cut-off comparisons with it. |
| 3 | **High** | `LSM6DS3.h` / `fac_imu.h` | `int16_t gyro_offsets[]` was a flexible array member, so it contributed 0 to `sizeof(LSM6DS3)` (28 bytes) and had no storage — yet `LSM6DS3` is embedded **by value** inside `Gyro`. Verified against the committed build: `gyro` sat at `0x200003e0` with `sizeof(Gyro) == 0x20`, `gyro_status` at offset 28, and `.bss.motors` immediately after at `0x20000400`. `LSM6DS3_calculate_offset()` stored `[0]`→bytes 28-29 (**over `gyro_status`**), `[1]`→30-31 (padding), `[2]`→32-33 (**past `gyro`, onto the low half-word of `motors[0]`, the Motor 1 pointer**). Latent only by luck of init ordering: `FAC_IMU_GET_status()` is read before the calibration and `FAC_motor_init()` reassigns `motors[0]` right after it. Also a C99/C11 constraint violation (§6.7.2.1p3) that GCC accepts silently. | The array is now sized: `int16_t gyro_offsets[LSM6DS3_AXIS_NUMBER]`, with the new `LSM6DS3_AXIS_NUMBER` (3) also replacing the hardcoded `3` in the three axis loops. `sizeof(LSM6DS3)` 28 → 36, `sizeof(Gyro)` 32 → 40; 8 bytes of RAM, no signature or protocol change. |
| 4 | **Medium** | `fac_settings.c` `FAC_settings_SET_value()` and the USB send helpers | The setting code arrives straight from USB (`comSerialBuffer[1]`, so `0…255`) and was never checked against `FAC_SETTINGS_CODE_LAST` (64), giving an out-of-bounds read/write on `settings[]` from a single malformed 4-byte packet. `Setting` is 8 bytes and the committed build placed `.data.settings` at `0x20000000`, the very first object in RAM, so `settings[code].value` reached `SystemCoreClock` at code 64, `uwTickPrio` at 65, and the USB CDC configuration descriptors from 66 on — and the value stored was arbitrary anyway, since the clamp read a garbage `min`/`max`. `FAC_settings_GET_value()` already checked. | The same guard added to all three: `if (code >= FAC_SETTINGS_CODE_LAST) return;`. An unknown code is now ignored on write and gets no reply on the two read commands. The `WRITE` ack byte is unchanged — `command_response()` still sends it — since a NACK convention would be a FAC Tool protocol change. |
| 6 | **Medium** | `fac_jingles.c` | Called `FAC_motor_make_noise()` without including `FAC_Code/fac_motors.h`; it compiled only via implicit declaration (a `-Wall` warning, and an error under C99+ strictness). | Fixed by the author: `#include "FAC_Code/fac_motors.h"` added to `fac_jingles.c`. |
| 7 | **Low** | `fac_servo.c` `FAC_servo_apply_settings()` | `((max-min)/100) * position / 10` truncated the span before scaling, losing up to ~100 µs of travel at full deflection, and a span below 100 µs made `span/100` evaluate to `0` — pinning the servo at `min_us` for every stick position. The settings ranges permit a span as small as 2 µs (`min ≤ 1499`, `max ≥ 1501`). The `/100` was there to avoid a 16-bit overflow, since `span × position` can reach ≈ 2.8 M. The default 1000/2000 configuration hid the bug entirely: `span/100 = 10` exactly, so the result was exact. | Single division on a 32-bit intermediate: `p = ((uint32_t) span * position) / MAX_SERVO_VALUE`. `MAX_SERVO_VALUE` was `RECEIVER_CHANNEL_RESOLUTION-1` **without parentheses** — harmless in the existing comparison/subtraction uses but wrong as a divisor, so it is now parenthesised. At full deflection the default config now reaches `CCR = 2000` instead of 1999. |
| 9 | **Low** | `fac_settings.c` | `FAC_settings_uint16_to_bytes()` wrote MSB-first while `FAC_settings_bytes_to_uint16()` read LSB-first, and the latter's comment claimed MSB-first. They were not inverses, and the single caller compensated by building a swapped temporary (`{comSerialBuffer[3], comSerialBuffer[2]}`). Harmless but a trap for anyone using them as a pair. | `bytes_to_uint16()` is now MSB-first (`array[0] << 8 | array[1]`), matching its comment and the wire format, so the two are exact inverses. The caller passes `&comSerialBuffer[2]` straight through — no temporary, no swap. |
| 10 | **Low** | `fac_app.c` cut-off check | `BATTERY_TYPE_NONE` has the value 5 and was used directly as a cell-count multiplier (`CUTOFF_VOLTAGE_MV × batteryType`), giving a 14000 mV threshold that can never be reached — so an unrecognised pack forced CUTOFF after the detection time. Safe, but accidental. | The multiplication is gone: the per-cell threshold is now compared against the per-cell voltage, and the three special cases are explicit — threshold `0` (user-disabled) and `BATTERY_TYPE_USB` always reset the timer, `BATTERY_TYPE_NONE` never does, so an unknown pack still ends in CUTOFF but **on purpose**. The enum values themselves are unchanged (the FAC Tool depends on them). |
| 11 | **Low** | `fac_functions.c` `FAC_functions_update_inputs()` | Did not reset disabled slots to `0.0f`, unlike the equivalent mix function, so a stale input could persist. No observable effect: the input channels only change through `FAC_functions_init()`, which zeroes every input, so a disabled slot always read `0.0f` anyway. | `else { FAC_functions_SET_input(i, 0.0f); }` added, matching `FAC_mixes_update_mix_inputs()`. |
| 12 | **Low** | `fac_mixes.c` | `mix_input_reversed[]` was populated in `init()` but never read — `FAC_mixes_GET_input_reversed()` had no callers at all (a `-Wunused-function` warning), and `FAC_mixes_update_mix_inputs()` re-read the setting on every input, every loop. | The cached field is now the one used: `if (FAC_mixes_GET_input_reversed(i))`. That drops 8 `FAC_settings_GET_value()` calls per loop and makes the flag consistent with `mix_input_channels_number`, which was already cached. **Behaviour note**: reversal now takes effect on *apply* rather than on the USB write, exactly like the input channel already did. |
| 13 | **Low** | `fac_mixes.h` | `FAC_mixes_update_mix_outputs()` was declared with empty parentheses but defined taking `float[]` — legal C but no type checking at call sites, and a hard error under C23 where `()` means `(void)`. | Declared as `void FAC_mixes_update_mix_outputs(float mix_output[]);`. |
| 14 | **Low** | `fac_std_receiver.c` | `FAC_std_receiver_new_channel_value()` compared the pre-deadzone input against the post-deadzone stored value, so it reported "out of range" whenever the deadzone changed the value — which, with any non-zero deadzone, is almost always. Every caller ignored the return. | The function returns `void`. The two receiver backends' `@note` lines referring to the return value were removed. |
| 15 | **Info** | `fac_motors.h`, `fac_servo.h` | `FAC_motor_GET_*` and `FAC_servo_GET_servo_freq()` were non-static but absent from the headers — unusable without a manual `extern`. | Declared in their headers, matching the other modules (`fac_servo`, `fac_battery` and `fac_app` all expose their `GET_` accessors). `FAC_motor_GET_reverse()` also returned `uint16_t` for a `uint8_t` field; it now returns `uint8_t`. |
| 16 | **Info** | `fac_battery.h` | `FAC_battery_GET_voltage()` and `FAC_battery_SET_calibration_offset()` were each declared twice; `FAC_battery_GET_type()` returned `uint16_t` for a `uint8_t` value. | Duplicates removed, return type narrowed to `uint8_t` in both the header and `fac_battery.c`. Callers were already assigning it to `uint8_t`. |
| 17 | **Medium** | `fac_mapper.c`, `fac_mixes.c`, `fac_functions.c` | Found while investigating #8, not part of the original list. The mapper settings have a single range of `0…210`, but only `100…109` (mix outputs) and `200…210` (function outputs) decode to something real. A value in **`110…199`** passed validation, entered the mix branch, and produced `FAC_mixes_GET_output(10…99)` — up to 90 floats read past the end of a 10-element array. The garbage float became a motor speed (`\|val\| × 1000`, then clamped to `MAX_DMA_PWM_VALUE`), so a mapper value of e.g. 150 could **spin a motor at full speed** in an arbitrary direction, and the value persisted to EEPROM. | `FAC_mixes_GET_output/_GET_input` and `FAC_functions_GET_output/_GET_input/_SET_output` bounds-check their index and return `0.0f` (or ignore the write) when it is out of range, so an invalid link resolves to "no output" for every caller. `FAC_mapper_apply_to_devices()` additionally guards its `functionsUpdated[]` index. The settings range is unchanged — it cannot express a gap, which is why the check belongs in the accessors. |
| 18 | **Info** | `fac_mapper.c` `FAC_mapper_apply_to_devices()` | Found later, outside the original list. The loop over the link array used `for (int i = 0; i < sizeof(links); i++)` on `uint8_t links[5]`, i.e. a **byte count** used as an element count. Correct only because the element size happens to be 1 byte; changing the array's type to anything wider would silently iterate past its end and read `functionsUpdated[]` indices out of a stale stack. | The element count is now explicit: `i < sizeof(links) / sizeof(links[0])`, the same idiom already used in `DMApwm.c`. Same 5 iterations today, but correct for any element type. |

### 11.2 Withdrawn (not bugs)

| # | Location | Was reported as | Why it is not a bug |
|---|---|---|---|
| 5 | `fac_motors.c` `FAC_motor_make_noise()` | The `for` loop of `HAL_Delay(0)` consumes the whole `duration`, so the audible part is near-zero while the call blocks for ~2× `duration`. | Verified on hardware: the jingles sound correct. The report assumed the tone comes from the direction-toggling `while` loop, but **the tone is the soft-PWM repetition rate itself** — `FAC_DMA_pwm_change_freq(freq)` drives the coils with a 5 % duty pulse train at `freq`, which keeps sounding until the frequency and speed are restored. So the note is heard *during* the supposedly wasted wait loop, for its full `duration`. Total blocking is ≈ `duration`, not 2×, and the direction loop only gets the leftover (≤ ~1 ms of a 125 ms note). Removing the wait loop would have shortened every note to nothing and hammered the IN/IN driver with kHz-rate reversals. The mechanism is now documented in [§ 8.9](#89-fac_motors--dc-motor-outputs) and in a comment above the function. |

---

*© The Floppy Lab™ — F.A.C. V2 firmware.*
