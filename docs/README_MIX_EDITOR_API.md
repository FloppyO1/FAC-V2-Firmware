# F.A.C. V2 — Mix & Special Function Authoring API

The API contract that a **generated** mix or special function must respect, written for whoever builds the planned graphical mix/function editor (see [CLAUDE.md § Planned: graphical mix/function editor](../CLAUDE.md)).

This is a **reference document**: it describes the code the tool has to emit, everything that code is allowed to call, what each call costs, and how a new mix or function is registered into the firmware. It does **not** design the editor itself — no UI, no block palette, no simulator architecture. Those decisions are free; the contract below is not.

> Companion documents: [README_API.md](README_API.md) (whole firmware architecture + API) and [CLAUDE.md](../CLAUDE.md) (build, conventions, known issues).

---

## Table of contents

1. [Scope and sources of truth](#1-scope-and-sources-of-truth)
2. [Execution model — what runs, when, how often](#2-execution-model--what-runs-when-how-often)
3. [The value contract](#3-the-value-contract)
4. [Anatomy of a generated mix](#4-anatomy-of-a-generated-mix)
5. [Anatomy of a generated special function](#5-anatomy-of-a-generated-special-function)
6. [Framework API](#6-framework-api)
7. [Math API — `fac_math.h`](#7-math-api--fac_mathh)
8. [Sensor and system API](#8-sensor-and-system-api)
9. [State and time](#9-state-and-time)
10. [Hard rules for generated code](#10-hard-rules-for-generated-code)
11. [Registration — the part a file generator cannot do](#11-registration--the-part-a-file-generator-cannot-do)
12. [Compatibility and versioning](#12-compatibility-and-versioning)
13. [Conformance checklist](#13-conformance-checklist)

---

## 1. Scope and sources of truth

The tool produces a `.c` / `.h` pair that drops into `Core/Src/FAC_Code/mixes_functions/mixes/` (or `.../functions/`) and compiles unchanged. Everything it may call is listed in this document.

**If this document and the source disagree, the source wins.** The files that define the contract:

| Source of truth | What it fixes |
|---|---|
| `Core/Inc/FAC_Code/mixes_functions/fac_math.h` | Every math primitive, its exact integer semantics, its cost |
| `Core/Inc/FAC_Code/mixes_functions/fac_mixes.h` | Mix framework API, `FAC_MIXES_ID`, input/output counts |
| `Core/Inc/FAC_Code/mixes_functions/fac_functions.h` | Special-function framework API, `FAC_SPECIAL_FUNCTIONS_ID` |
| `Core/Src/FAC_Code/mixes_functions/mixes/fac_template_mix.c.template` | The mix boilerplate, verbatim |
| `Core/Src/FAC_Code/mixes_functions/functions/fac_template_function.c.template` | The special-function boilerplate, verbatim |
| `Core/Inc/FAC_Code/config.h` | `RECEIVER_CHANNEL_RESOLUTION`, `RECEIVER_CHANNELS_NUMBER`, resolutions |
| `Core/Inc/FAC_Code/fac_imu.h`, `fac_battery.h`, `fac_adc.h`, `fac_std_receiver.h`, `fac_app.h`, `fac_settings.h` | The sensor and system accessors of § 8 |

The two committed implementations are the **conformance examples** — a generated file should be indistinguishable in shape from them:

- `Core/Src/FAC_Code/mixes_functions/mixes/fac_simple_tank_mix.c`
- `Core/Src/FAC_Code/mixes_functions/functions/fac_direct_link_function.c`

---

## 2. Execution model — what runs, when, how often

Bare-metal superloop, no RTOS, no preemption. One main-loop pass takes about **1 ms** (≈ 1 kHz). A mix or special function is a plain function called from that loop; there is no scheduler, no task, no callback.

The call chain is:

```
FAC_app_main_loop()
  └── case FAC_STATE_NORMAL:            <-- ONLY in this state
        └── FAC_mapper_apply_to_devices()
              ├── FAC_mix_update()            if any device links to 100+i
              │     └── FAC_<name>_mix_update()
              └── FAC_functions_update(n)     once per distinct linked function n
                    └── FAC_<name>_function_update(n)
```

Five rules follow from that chain, and every one of them matters to a generated graph that holds state:

1. **Only `FAC_STATE_NORMAL` runs the chain.** In `FAC_STATE_DISARMED` and `FAC_STATE_CUTOFF` the mapper is never called, so the mix and every special function are simply **not executed**. Motors are forced to speed 0 and servos disabled directly by the state handler. A latch, a timer or a filter therefore **freezes** across a disarm and resumes with values that may be seconds or minutes old — see [§ 9.4](#94-the-disarm-gap).
2. **A mix runs only if at least one device links to it** (`FAC_SETTINGS_CODE_MAPPER_M1..S2` holding `100+i`). If the user maps nothing to the mix, the mix never executes at all.
3. **A mix runs at most once per pass**, however many devices link to it — the mapper tracks it with a local `mixUpdated` flag. All ten outputs come from that single call.
4. **A special function runs at most once per pass**, tracked per slot in `functionsUpdated[]`. A function whose slot no device links to never executes.
5. **The whole chain is synchronous and blocking.** Whatever a generated function does is paid for inside that ~1 ms budget, on a 48 MHz Cortex-M0 with **no FPU and no hardware divider**.

**Watchdog.** The IWDG timeout is **~400 ms** and is refreshed once per main-loop pass — *after* the mapper has returned. Generated code must therefore never block: no `HAL_Delay`, no wait loop, no polling on a peripheral. A blocking generated mix resets the board.

**Budget.** There is no enforced limit, but the mapper's own measured share of the pass is the yardstick. If a graph starts costing more than a few hundred microseconds, profile it on target with `FAC_debug_utils` (`CRONOMETER_FAC_MAPPER`, see [README_API.md § 8.19](README_API.md#819-fac_debug_utils--on-target-profiling)) rather than guessing. The single most useful cost rule is in [§ 7](#7-math-api--fac_mathh): **count the divisions.**

---

## 3. The value contract

### 3.1 The type and the scale

```c
typedef int32_t fac_value_t;

#define FAC_VALUE_MAX   1000     // full scale
#define FAC_VALUE_MIN  (-1000)   // full scale in the opposite direction
#define FAC_VALUE_ZERO  0        // motor stopped / servo centered
```

`int32_t` and not `int16_t` deliberately: the M0 works on 32-bit registers, so a narrower type costs a sign-extension on nearly every operation.

The `±1000` scale is **not arbitrary**. It equals `RECEIVER_CHANNEL_RESOLUTION`, `MOTOR_SPEED_RESOLUTION` and `SERVO_POSITION_RESOLUTION` (all `1000`, all in `config.h`), so the conversions at both ends of the chain are exact and nothing is rounded away.

### 3.2 What a value means at the device end

The mapper converts a normalized output into device units. A generated graph does not do this itself — it only has to know what its number will mean:

| Device | Conversion applied by `fac_mapper.c` | Meaning |
|---|---|---|
| DC motor (M1–M3) | `dir = (v < 0) ? BACKWARD : FORWARD`, `speed = FAC_math_abs(v) * 1000 / 1000` | **sign is direction, magnitude is speed**. `0` is stopped, `±1000` is full speed |
| Servo / ESC (S1–S2) | `position = FAC_math_to_range(v, 0, SERVO_POSITION_RESOLUTION)` | `-1000` → position `0` (one end of travel), `+1000` → position `1000` (the other end). For a servo that is 0°…180°, for an ESC 0 %…100 % |

An output the user does not use must be left at `FAC_VALUE_ZERO` — the boilerplate already does that, see [§ 4](#4-anatomy-of-a-generated-mix).

### 3.3 How an input is built

Both frameworks build their inputs the same way, from a receiver channel selected by a setting:

```c
uint16_t rxValue = FAC_std_receiver_GET_channel(chNumber);   // 0 .. 999, deadzone already applied
fac_value_t in   = FAC_math_from_range(rxValue, 0, RECEIVER_CHANNEL_RESOLUTION);
```

With both resolutions equal to 1000 this resolves to the exact `2*rx - 1000`, with no float and no rounding. Two consequences the tool must model:

- **A channel value of `0` is not "no signal", it is full negative travel** (`-1000`). The firmware has a separate connection gate (`FAC_std_receiver_GET_is_connected()`); an input value never says "no signal" by itself.
- A slot whose input-channel setting is `0` is **disabled**, and its input is forced to `FAC_VALUE_ZERO`, not left stale.

For a mix, each of the 8 inputs additionally has a **reversal** flag; when set the value is negated (`inputValue = -inputValue`) after normalization. Both the channel number and the reversal flag are **cached by `FAC_mixes_init()`**, so a change through the FAC Tool takes effect on the next *apply settings*, not on the USB write itself.

### 3.4 Exactness rules — the ones a PC simulator must mirror

These are the rules that make the browser and the MCU produce **bit-identical** results. They are written at the top of `fac_math.h` too.

| Rule | Detail |
|---|---|
| **Integer division truncates toward zero** | `-7 / 2` is `-3`, **not** `-4`. In JavaScript that is `Math.trunc(a / b)`, **never** `Math.floor`. This is the classic difference that diverges silently, and it appears in almost every primitive that costs a division |
| **The `%` operator follows the same sign rule** | `-7 % 2` is `-1` in C, and in JavaScript. Consistent, but do not assume Python semantics |
| **Group 1 always saturates** | Every group-1 primitive clamps its result to `[-1000, +1000]`, and most clamp their *arguments* first. The order matters: `FAC_math_mul(a, b)` is `clamp(clamp(a) * clamp(b) / 1000)`, so an out-of-range argument is clipped **before** the product |
| **Group 2 does not saturate** | The raw fixed-point primitives work on the whole `int32_t` range on purpose |
| **Every intermediate must fit in `int32_t`** | Signed overflow is undefined behaviour in C, and a simulator using JavaScript doubles would *not* reproduce it. Where a primitive has an input-range limit it is stated in [§ 7](#7-math-api--fac_mathh); the tool must enforce those limits at graph-validation time, not at run time |
| **`int32_t` wrap is not a feature** | Do not design a block that relies on wrapping. The one legitimate wrap is `HAL_GetTick()` arithmetic, see [§ 9.3](#93-time) |

---

## 4. Anatomy of a generated mix

A mix has **up to 8 inputs and 10 outputs**, and **exactly one mix is active at a time** (`FAC_SETTINGS_CODE_ACTIVE_MIX`). It is the right shape for coupled logic: differential steering, a melty-brain translation, anything where several outputs are computed together from shared state.

### 4.1 The `.c` file

The boilerplate around `/* INSERT YOUR CODE HERE */` is **not negotiable** — the framework depends on it. The generator fills in only the marked regions.

```c
/*
 * fac_<name>_mix.c
 *
 *  <the 9-step recipe comment block, copied verbatim from the template>
 *
 *  Created on: <date>
 *      Author: <author>
 */

#include "FAC_Code/mixes_functions/mixes/fac_<name>_mix.h"     /* GENERATED: file name */

/* DEFAULT INCLUDE */ // DON´T TOUCH THIS CODE!!
#include "FAC_Code/mixes_functions/fac_mixes.h"
#include "FAC_Code/fac_settings.h"

/* CUSTOM INCLUDE */                                            /* GENERATED: only what the graph uses */
//#include "FAC_Code/fac_adc.h"
//#include "FAC_Code/fac_imu.h"      // if get_status == HAL_ERROR NOT USE data!!

/* PRIVATE FUNCTIONS AND VARIABLES */
// marker only, it records which ID this file implements (see step 5): unused on purpose
static const uint8_t __attribute__((unused)) mix_id = FAC_MIX_<NAME>;   /* GENERATED: enum id */

/* WHAT THIS MIX DO */
/*
 * DESCRIPTION:
 * <GENERATED: human description of the graph>
 * ...
 */
#define INPUT_<NAME>  0        /* GENERATED: one per used input, 0..7,  names are free */
#define OUTPUT_<NAME> 0        /* GENERATED: one per used output, 0..9, names are free */

/*
 * @brief	Calculate the mix output values
 */
void FAC_<name>_mix_update(void) {                              /* GENERATED: function name */
	// this code must be left as it is, DON'T TOUCH IT!
	fac_value_t outputs[MIXES_MAX_OUTPUTS_NUMBER];
	fac_value_t inputs[MIXES_MAX_INPUTS_NUMBER];
	FAC_mixes_update_mix_inputs();
	for (int i = 0; i < MIXES_MAX_OUTPUTS_NUMBER; i++) {
		outputs[i] = FAC_VALUE_ZERO;     // every output starts at zero, the unused ones stay there
	}
	for (int i = 0; i < MIXES_MAX_INPUTS_NUMBER; i++) {
		inputs[i] = FAC_mixes_GET_input(i);
	}
	/* INSERT YOUR CODE HERE -START- */

	/* GENERATED: the compiled graph.
	 * Reads inputs[INPUT_*], writes outputs[OUTPUT_*]. */

	/* INSERT YOUR CODE HERE -END- */
	// keep outputs in range
	for (int i = 0; i < MIXES_MAX_OUTPUTS_NUMBER; i++) {
		outputs[i] = FAC_math_clamp(outputs[i]);
	}
	// update outputs values on mixes struct
	FAC_mixes_update_mix_outputs(outputs);
}
```

What the generator may vary, and nothing else:

| Region | Rule |
|---|---|
| File name and the `#include` of its own header | Must match: `fac_<name>_mix.c` / `.h` |
| `/* CUSTOM INCLUDE */` block | Add only the headers the graph actually uses ([§ 8](#8-sensor-and-system-api)). An unused include is harmless; a missing one is a `-Wall` implicit-declaration warning. ⚠ **The boilerplate does not include `main.h`**, so `TRUE` / `FALSE` are *not* available by default — a graph using them must add `#include "main.h"` here, or use `1` / `0`. Same for `abs()`, which needs `stdlib.h` (and `FAC_math_abs` should be preferred anyway, since it clamps) |
| `mix_id` | A **documentation marker only**, deliberately never read. Keep `static const uint8_t` and keep `__attribute__((unused))` — without it `-Wall` warns. The `case` label in the dispatcher uses the enum name directly, because a `static const` from another translation unit is neither visible nor a valid `case` label |
| The `DESCRIPTION` / `INPUTs` / `OUTPUTs` comment block | Free text, but it is the only human-readable record of what the graph does. Emit a real description |
| `#define INPUT_*` / `OUTPUT_*` | Names are free, **the numbers are positional and must not be renumbered**. Input *i* is `inputs[i]`, output *i* is `outputs[i]`, both **0-based** |
| Function name | `FAC_<name>_mix_update(void)` — must match the header and the dispatcher `case` |
| The region between the two `INSERT YOUR CODE HERE` markers | The compiled graph. **This is the only executable part the tool owns** |
| File-scope `static` variables, above the function | Allowed and necessary for stateful blocks, see [§ 9](#9-state-and-time) |

### 4.2 The `.h` file

```c
#ifndef INC_FAC_CODE_MIXES_FUNCTIONS_FAC_<NAME>_MIX_H_
#define INC_FAC_CODE_MIXES_FUNCTIONS_FAC_<NAME>_MIX_H_

#include "stm32f0xx_hal.h"

void FAC_<name>_mix_update(void);

#endif /* INC_FAC_CODE_MIXES_FUNCTIONS_FAC_<NAME>_MIX_H_ */
```

The include guard must be unique — derive it from the file name, as above. Declare the update function with `(void)`, not `()`: empty parentheses mean "unspecified arguments" in C11 and are a hard error under C23.

---

## 5. Anatomy of a generated special function

A special function has **exactly 1 input and 1 output**, and **up to 20 can run simultaneously**. It is the right shape for independent per-channel behaviour: a pass-through, a shaped curve, a ramp on one servo.

`SPECIAL_FUNCITONS_NUMBER` is `20` (note the spelling — it is the real identifier). `FAC_SPECIAL_FUNCTION_LAST` must never exceed it.

### 5.1 The `.c` file

```c
#include "FAC_Code/mixes_functions/functions/fac_<name>_function.h"

/* DEFAULT INCLUDE */ // DON´T TOUCH THIS CODE!!
#include "FAC_Code/mixes_functions/fac_functions.h"
#include "FAC_Code/fac_settings.h"

/* CUSTOM INCLUDE */
//#include "FAC_Code/fac_imu.h"      // if get_status == HAL_ERROR NOT USE data!!

/* PRIVATE FUNCTIONS AND VARIABLES */
// marker only, it records which ID this file implements (see step 5): unused on purpose
static const uint8_t __attribute__((unused)) first_special_function_id =
		FAC_SPECIAL_FUNCTION_<NAME>_1ST;

/* WHAT THIS SPECIAL FUNCTION DO */
/*
 * DESCRIPTION: <GENERATED>
 * INPUTs DESCRIPTION: <GENERATED>
 * OUTPUTs DESCRIPTION: <GENERATED>
 */

void FAC_<name>_function_update(uint8_t sFunctionID) {
	// this code must be left as it is, DON'T TOUCH IT!
	uint8_t functionArrayPosition = sFunctionID;
	FAC_functions_update_input(functionArrayPosition);
	fac_value_t input = FAC_functions_GET_input(functionArrayPosition);
	fac_value_t output = FAC_VALUE_ZERO;
	/* INSERT YOUR CODE HERE -START- */

	/* GENERATED: reads `input`, writes `output`. */

	/* INSERT YOUR CODE HERE -END- */
	// keep outputs in range
	output = FAC_math_clamp(output);
	// update outputs values on mixes struct
	FAC_functions_SET_output(functionArrayPosition, output);
}
```

### 5.2 Multiple instances — the rule that bites

One `.c` file can serve several enum IDs, which is how `direct link` covers eight slots with one implementation. The parameter `sFunctionID` **is** the array position of the slot being computed.

If the tool emits a multi-instance function it must:

- add **consecutive** IDs with `1ST` / `2ND` / `3RD`… suffixes to `FAC_SPECIAL_FUNCTIONS_ID`;
- group their `case` labels in `FAC_functions_update()` with **no `break` except on the last one**;
- set `first_special_function_id` to the **first** of them;
- **key every `static` state variable by `sFunctionID`**, not keep one shared variable. A stateful multi-instance function needs `static int32_t state[SPECIAL_FUNCITONS_NUMBER];` (or an array sized to its own instance count, indexed by `sFunctionID - FIRST_ID`), otherwise all instances share one latch and interfere. This is the single most likely defect in generated multi-instance code.

---

## 6. Framework API

### 6.1 Available to a mix — `fac_mixes.h`

```c
#define MIXES_MAX_INPUTS_NUMBER  8
#define MIXES_MAX_OUTPUTS_NUMBER 10

void        FAC_mixes_update_mix_inputs(void);
void        FAC_mixes_update_mix_outputs(fac_value_t mix_output[]);
fac_value_t FAC_mixes_GET_input (uint8_t inputNumber);    // 0-based
fac_value_t FAC_mixes_GET_output(uint8_t outputNumber);   // 0-based
```

| Function | Semantics | Who calls it |
|---|---|---|
| `FAC_mixes_update_mix_inputs` | Refreshes all 8 inputs from the receiver: normalizes `0…999` → `-1000…+1000`, applies the cached per-input reversal, forces a disabled input (channel `0`) to `FAC_VALUE_ZERO` | **The boilerplate**, once, at the top. Never call it from graph code |
| `FAC_mixes_GET_input(i)` | The normalized value of input *i*. Bounds-checked: `i >= 8` returns `FAC_VALUE_ZERO` | The boilerplate fills `inputs[]`; graph code reads `inputs[i]` |
| `FAC_mixes_update_mix_outputs(out)` | Copies the local `outputs[]` into the shared `Mixes` struct the mapper reads. **Must be the last statement** | **The boilerplate**, once, at the end |
| `FAC_mixes_GET_output(i)` | Reads back a published output. Bounds-checked. Of no use to graph code — the local `outputs[]` array holds the current pass's values, the struct still holds the previous pass's until the write-back |

`extern int32_t map_int32(...)` is also declared in `fac_mixes.h` for an arbitrary range conversion, but `FAC_math_to_range` / `FAC_math_from_range` cover the normalized cases and should be preferred.

### 6.2 Available to a special function — `fac_functions.h`

```c
#define SPECIAL_FUNCITONS_NUMBER 20      /* note the spelling, it is the real identifier */

void        FAC_functions_update_input (uint8_t functionNumber);
void        FAC_functions_update_inputs(void);
fac_value_t FAC_functions_GET_input (uint8_t functionNumber);
fac_value_t FAC_functions_GET_output(uint8_t functionNumber);
void        FAC_functions_SET_output(uint8_t functionNumber, fac_value_t outputValue);
```

| Function | Semantics |
|---|---|
| `FAC_functions_update_input(n)` | Refreshes **one** slot from the receiver. Called by the boilerplate with the function's own slot. Out-of-range index ignored; a disabled slot is reset to `FAC_VALUE_ZERO` |
| `FAC_functions_update_inputs()` | Refreshes all 20 slots. **Has no caller today**, kept for a function that has to read slots other than its own — call it *before* reading them |
| `FAC_functions_GET_input(n)` / `_GET_output(n)` | Bounds-checked reads of the shared arrays; out of range returns `FAC_VALUE_ZERO`. Reading another slot's **output** gives the value from the *previous* pass unless that function already ran this pass — the mapper's update order is the order of the five link settings (M1, M2, M3, S1, S2), which is not a dependency graph. **A generated function must not depend on another function's output** |
| `FAC_functions_SET_output(n, v)` | Where the function publishes its result. Called by the boilerplate |

### 6.3 What generated code must **not** call

`FAC_motor_set_speed_direction()`, `FAC_servo_set_position()`, `FAC_servo_enable/disable()`, `FAC_motor_make_noise()` and every other device setter belong to the **mapper**, which rewrites them on every pass. A mix that writes a device directly is overwritten a few microseconds later at best, and fights the mapper's safety behaviour at worst. Likewise nothing in generated code may write a setting or touch the EEPROM.

---

## 7. Math API — `fac_math.h`

Header-only, `static inline`, no floats anywhere. **This is the closed set of operations the tool composes graphs out of** — it is what makes browser simulation possible at all, because twenty-one known integer operations can be reproduced exactly while arbitrary C cannot. (Twenty-two are defined; `FAC_math_atan_ratio` is a helper of `atan2` a graph has no reason to call, see [§ 7.5](#75-group-3--angles-and-trigonometry).)

### 7.1 The cost model

The Cortex-M0 has no hardware divider **and** no long multiply (`umull`), which is what a compiler would need to turn a division by a constant into a multiply-and-shift. So even a plain `/ 1000` compiles to a call to `__aeabi_idiv`. **Integer division is the expensive operation**; the only exception is a division by a power of two, which folds to a shift.

| Divisions | Primitives |
|---|---|
| **0** | `clamp`, `abs`, `add`, `sub`, `min`, `max`, `clamp_to`, `angle_wrap` |
| **0, but not free** | `sqrt` — no division and no library call whatsoever, but a fixed **16 iterations** of shift/compare/subtract |
| **1** | `mul`, `scale`, `blend`, `deadzone`, `to_range`, `from_range`, `mul_scaled`, `div_scaled` |
| **3** | `expo` |
| **4** | `sin`, `cos` |
| **6** | `atan2` |

A graph built only out of the zero-division primitives costs no division at all — which covers most mixes. `fac_simple_tank_mix.c` is one of them. **`sqrt` is the one row not to read as a cost**: it pays no division, but it always walks its 16 steps, so budget it like a couple of divisions rather than like a `clamp`.

### 7.2 Constants

```c
#define FAC_VALUE_MAX  1000
#define FAC_VALUE_MIN  (-1000)
#define FAC_VALUE_ZERO 0

#define FAC_ANGLE_TURN         4096                        /* a full turn, binary angle */
#define FAC_ANGLE_MASK         (FAC_ANGLE_TURN - 1)
#define FAC_ANGLE_HALF_TURN    (FAC_ANGLE_TURN / 2)        /* 180 deg */
#define FAC_ANGLE_QUARTER_TURN (FAC_ANGLE_TURN / 4)        /*  90 deg */
#define FAC_MATH_DEG(d)        (((int32_t)(d) * FAC_ANGLE_TURN) / 360)
#define FAC_MATH_FINE          (FAC_VALUE_MAX * 10)        /* 10000, internal trig scale */
```

A full turn is **4096 units** (one unit = 0.088°) because that makes wrapping a single `AND` instead of a modulo — which on a core without a divider would be a library call. `FAC_MATH_DEG(90)` lets angles be written in degrees; it is a compile-time constant when its argument is one.

### 7.3 Group 1 — normalized values

All of these operate on `[-1000, +1000]` and **saturate**. This is what a mix uses for almost everything.

| Primitive | Exact behaviour | Div | Constraints |
|---|---|---|---|
| `FAC_math_clamp(v)` | `v` limited to `[-1000, +1000]` | 0 | — |
| `FAC_math_abs(v)` | `\|clamp(v)\|`, result in `[0, 1000]` | 0 | Clamps **before** taking the magnitude |
| `FAC_math_add(a, b)` | `clamp(clamp(a) + clamp(b))` | 0 | Saturating sum — full throttle plus full steering gives full scale, not an overflow |
| `FAC_math_sub(a, b)` | `clamp(clamp(a) - clamp(b))` | 0 | |
| `FAC_math_mul(a, b)` | `clamp(clamp(a) * clamp(b) / 1000)` | 1 | Both operands are fractions of full scale, so the product is `a*b/1000`: `1000` behaves as "×1", `500` as "×0.5". Use it to modulate one value by another |
| `FAC_math_scale(v, percent)` | `percent` first clipped to `[-10000, +10000]`, then `clamp(clamp(v) * percent / 100)` | 1 | `percent` is a plain percentage: `100` leaves the value alone, `50` halves it, `200` doubles it (then saturates), negative also reverses. The `±10000` clip is what keeps the product inside `int32_t` |
| `FAC_math_min(a, b)` | `min(clamp(a), clamp(b))` | 0 | |
| `FAC_math_max(a, b)` | `max(clamp(a), clamp(b))` | 0 | |
| `FAC_math_blend(a, b, w)` | `w <= 0` → `clamp(a)`; `w >= 1000` → `clamp(b)`; else `clamp((clamp(a)*(1000-w) + clamp(b)*w) / 1000)` | 1 | `w` is 0 (all `a`) … 1000 (all `b`), 500 is the average. Fades between two behaviours with a channel instead of switching abruptly |
| `FAC_math_deadzone(v, size)` | `size <= 0` → `clamp(v)`; `size >= 1000` → `0`; `m = abs(v)`; `m <= size` → `0`; else `out = (m - size) * 1000 / (1000 - size)`, sign of `v` restored | 1 | `size` is the half-width of the dead band in normalized units — `50` ignores the first 5 % of travel. Outside the band the value is **re-stretched to full scale**, so the ends still reach `±1000`. The receiver already applies its own deadzone; this one is for a value the graph computed itself |
| `FAC_math_expo(v, amount)` | `v = clamp(v)`; `amount <= 0` → `v`; `amount` clipped to `1000`; `cube = ((v*v)/1000 * v)/1000`; result `clamp((v*(1000-amount) + cube*amount) / 1000)` | 3 | The classic RC expo: `0` linear … `1000` fully cubic. Fine control around the centre, full power still at the ends |
| `FAC_math_to_range(v, out_min, out_max)` | `clamp(v)` then `(v - (-1000)) * (out_max - out_min) / 2000 + out_min` | 1 | `-1000` maps to `out_min`, `+1000` to `out_max`. **`(out_max - out_min)` must stay below about 10⁶** |
| `FAC_math_from_range(x, in_min, in_max)` | `in_max == in_min` → `0`; `x` clipped into `[in_min, in_max]`; `(x - in_min) * 2000 / (in_max - in_min) + (-1000)` | 1 | `in_min` maps to `-1000`, `in_max` to `+1000`. The entry point for anything not already normalized — a raw sensor reading above all. **`(in_max - in_min)` must stay below about 10⁶** |

### 7.4 Group 2 — raw fixed point

These work on the whole `int32_t` range and **do not saturate**. They are the tools for processing sensor data: a gyroscope reading is ±32768 raw, and squeezing it into ±1000 before doing the math throws the signal away. **Normalize at the end**, when the result becomes an output, with `FAC_math_from_range`.

| Primitive | Exact behaviour | Div | Constraints |
|---|---|---|---|
| `FAC_math_mul_scaled(a, b, scale)` | `scale == 0` → `0`; else `a * b / scale` | 1 | `scale` is what represents "one": with `scale = 1000`, `a` and `b` are thousandths. **`a * b` must fit in `int32_t`** — keep both below about 46000 when they are of similar size |
| `FAC_math_div_scaled(a, b, scale)` | `b == 0` → `0`; else `a * scale / b` | 1 | **`a * scale` must fit in `int32_t`** |
| `FAC_math_clamp_to(v, min, max)` | `v` limited to `[min, max]` | 0 | The raw counterpart of `FAC_math_clamp`, which is fixed on `[-1000, +1000]` |
| `FAC_math_sqrt(v)` | `v <= 0` → `0`; else the largest `r` with `r*r <= v`, i.e. the real square root truncated toward zero | 0, **16 steps** | **Exact, not an approximation** — unlike the trigonometry of [§ 7.5](#75-group-3--angles-and-trigonometry). Verified against a reference over 439 116 values (every integer up to 300 000, plus the neighbourhood of every perfect square across the whole `int32_t` range) with no disagreement. Binary restoring method: shift, compare, subtract, so **not even an `__aeabi_idiv` call**. See the scaling rule below. **`v` must fit `int32_t` *after* the pre-multiplication that rule requires — under ~2×10⁹** |

Both guarded cases (`scale == 0`, `b == 0`) return `0` rather than trapping — and so does `sqrt` on a negative argument. A simulator must reproduce all three, not throw.

> **The scale halves — this is what a graph gets wrong.** `sqrt(x × scale)` is `sqrt(x) × sqrt(scale)`, so a root taken of a scaled value comes back on **half** the scale it went in on. To keep the result on the input's scale, multiply the input by that scale once more before calling: `FAC_math_sqrt(x * scale)` is `x`'s root, still on `scale`. The catch is the `int32_t` ceiling above, and that is precisely where the caller has to choose its units. For the centripetal speed of a melty brain (`ω = √(a/r)`, [§ 8.1](#81-imu--fac_codefac_imuh)): **centi-radians per second fit, milli-radians per second overflow.**

### 7.5 Group 3 — angles and trigonometry

No lookup table: integer polynomial approximations, so they cost no flash for data and every division is by a constant.

| Primitive | Exact behaviour | Div | Notes |
|---|---|---|---|
| `FAC_math_angle_wrap(angle)` | `angle & 4095` | 0 | Works on negatives too: `-512` comes back as `3584`, the same direction. Result in `[0, 4095]` |
| `FAC_math_sin(angle)` | Parabolic `4t(1-\|t\|)` plus the classic `0.225` correction, computed on the `FAC_MATH_FINE` scale and divided by 10 at the end | 4 | Returns a normalized value `[-1000, +1000]`. **Error ≤ 2 units out of 1000** over the whole turn, and exactly `0 / +1000 / 0 / -1000` at the quadrant points |
| `FAC_math_cos(angle)` | `FAC_math_sin(angle + FAC_ANGLE_QUARTER_TURN)` | 4 | |
| `FAC_math_atan_ratio(ratio)` | Helper of `atan2` covering the first eighth of a turn; `ratio` is on the fine scale (`10000` = 1.0) | 5 | Returns `[0, 512]`, i.e. 0°…45°. Public only because the header is header-only; **a graph should call `atan2`, not this** |
| `FAC_math_atan2(y, x)` | Full-turn arc tangent; `0` when both `x` and `y` are `0` | 6 | Result in `[0, 4095]`, `0` along `+x`, growing counterclockwise. `x` and `y` matter only by their **ratio**, so raw sensor counts can be passed directly. **Error ≤ 2 angle units (0.18°)** over 3600 measured directions. **Keep `\|x\|` and `\|y\|` below about 200000**: the ratio is `\|y\| * 10000 / \|x\|` and must stay inside `int32_t` — any sensor on this board is well within it |

For scale: a robot spinning at 20 turns per second covers about 18° between two iterations of a 400 Hz loop, so the loop rate limits the achievable heading a hundred times more than these approximations do.

---

## 8. Sensor and system API

Everything in this section is **outside** the mix/function framework. A graph that reads a sensor must include the module's header in the `/* CUSTOM INCLUDE */` block and handle the failure modes documented here.

### 8.1 IMU — `FAC_Code/fac_imu.h`

```c
HAL_StatusTypeDef FAC_IMU_GET_status(void);
void    FAC_IMU_update(void);                  /* the ONLY call that touches the I2C bus */
int16_t FAC_IMU_GET_accel_raw(uint8_t axis);   /* axis is X_AXIS / Y_AXIS / Z_AXIS */
int16_t FAC_IMU_GET_gyro_raw (uint8_t axis);
int32_t FAC_IMU_GET_accel_X_mg(void);          /* milli g,        1 g   = 1000 */
int32_t FAC_IMU_GET_accel_Y_mg(void);
int32_t FAC_IMU_GET_accel_Z_mg(void);
int32_t FAC_IMU_GET_gyro_X_mdps(void);         /* milli deg/s,    1 dps = 1000 */
int32_t FAC_IMU_GET_gyro_Y_mdps(void);
int32_t FAC_IMU_GET_gyro_Z_mdps(void);
```

**Three rules, all mandatory:**

1. **The main loop does not call `FAC_IMU_update()`.** Today its only caller is the telemetry path in `fac_settings.c`. A graph that reads the IMU **must call `FAC_IMU_update()` itself**, once, at the top of its generated region, before any accessor. Without it the accessors return whatever the last telemetry request left behind — data that is stale by an unbounded amount and only refreshes when the FAC Tool happens to be connected.
2. **Always check `FAC_IMU_GET_status() != HAL_ERROR` before trusting the data.** An init failure is non-fatal (the firmware boots without the IMU and blinks the LED 20 times), and a *runtime* read failure sets the same status while leaving the previous values in place. The status is the only thing that distinguishes fresh data from stale — a dead sensor reads exactly like a sensor holding perfectly still, which is the reading a self-righting or stabilisation graph would trust. **The generator must emit a status guard and a defined fallback** (typically: hold the last output, or fall back to `FAC_VALUE_ZERO`).
3. `FAC_IMU_update()` is **safe to call unconditionally** and cheap to call repeatedly. Three guards make it so: it returns immediately before the init chain completed; a second call **inside the same millisecond** returns at once (at the 416 Hz output data rate a new sample only lands every 2.4 ms, so every consumer of one loop pass shares one transaction); and after a failure it retries only **once a second**, so a sensor that dropped off the bus costs one 10 ms timeout per second instead of one per pass.

| Accessor | Units and range | Cost |
|---|---|---|
| `FAC_IMU_GET_accel_raw(axis)` | Raw counts, `±32768` at ±16 g full scale. Returns `0` for `axis >= 3` | free field read |
| `FAC_IMU_GET_gyro_raw(axis)` | Raw counts, `±32768` at ±2000 dps. Gyro offsets already applied and saturated by the driver | free field read |
| `FAC_IMU_GET_accel_*_mg()` | milli g, `±16000` at full scale (`raw × 488 / 1000`) | **1 division** each |
| `FAC_IMU_GET_gyro_*_mdps()` | milli deg/s, `±2000000` at full scale (`raw × 70`) | **0 divisions** |

Milli-degrees and not whole degrees on purpose: the sensitivity is a whole **70 mdps per count**, so the conversion is a pure multiplication and no resolution is thrown away on the slow rates. The accelerometer's 0.488 mg per count is not a whole number, which is why `_mg` costs a division.

> **A graph is usually better off with `_raw` plus `FAC_math_from_range`** than with the engineering-unit accessors: it normalizes in one step without paying for an intermediate unit conversion. Example — turn Z-axis rotation into a normalized value with a ±500 dps working range:
> ```c
> /* 500 dps at 70 mdps/count is 7142 counts */
> fac_value_t spin = FAC_math_from_range(FAC_IMU_GET_gyro_raw(Z_AXIS), -7142, 7142);
> ```

**Axis enum** (from `Libraries/LSM6DS3.h`): `X_AXIS = 0`, `Y_AXIS = 1`, `Z_AXIS = 2`.

> **Gyro full scale is ±2000 dps ≈ 333 RPM.** Enough for stabilisation or self-righting, but a melty brain saturates it well below working speed. The usual alternative is centripetal acceleration (`ω = √(a/r)`), bounded in turn by the ±16 g accelerometer and by how far from the spin axis the IMU sits. The integer square root a graph doing this needs is **`FAC_math_sqrt`** ([§ 7.4](#74-group-2--raw-fixed-point)) — exact, no division, a fixed 16 steps. What the graph has to get right is the **units**, because the root halves the scale: `sqrt(x * scale)` is `sqrt(x) * sqrt(scale)`, so to keep the result on the scale the input was in, the input must be multiplied by that scale once more before the call, and the product must still fit `int32_t` (under ~2×10⁹). For `ω = √(a/r)` that is exactly where the choice is made: **centi-radians per second fit, milli-radians per second overflow.**

### 8.2 Battery — `FAC_Code/fac_battery.h`

```c
uint16_t FAC_battery_GET_voltage(void);        /* pack voltage in mV, 8.02 V -> 8020 */
uint16_t FAC_battery_GET_cell_voltage(void);   /* single cell in mV */
uint8_t  FAC_app_GET_battery_type(void);       /* from fac_app.h — cell count detected at boot */
```

> ⚠ **`FAC_battery_GET_voltage()` is expensive and is not integer.** It re-reads the ADC **5 times** and computes the conversion in **floats** — on a core with no FPU, inside the 1 ms budget. It is fine on a slow path (a low-battery indication updated a few times a second) and wrong in a per-pass graph. If a graph needs the voltage, read it on a `HAL_GetTick()` interval and cache it, see [§ 9.3](#93-time). `FAC_battery_GET_cell_voltage()` calls it internally and costs the same, plus one division.

`enum BATTERY_TYPE`: `BATTERY_TYPE_USB = 0`, `_1S = 1`, `_2S = 2`, `_3S = 3`, `_4S = 4`, `_NONE = 5`. The values **double as the cell count** for 1S–4S — that is exactly why `USB` and `NONE` must be special-cased explicitly instead of being used in arithmetic. **These numeric values are FAC Tool ABI** (telemetry byte `[19]`) and must never be renumbered.

### 8.3 ADC — `FAC_Code/fac_adc.h`

```c
uint16_t FAC_adc_get_raw_channel_value(uint8_t chNumber);   /* 0 = VBAT, 1 = ADC_AUX, 2 = VREFINT */
uint16_t FAC_adc_GET_resolution(void);                      /* full-scale count, 4096 for 12 bit */
uint32_t FAC_adc_GET_Vref_in_uV(void);                      /* measured VDDA in microvolts */
```

The raw values come from a free-running DMA scan, so reading them costs nothing — this is the cheap way to get an analog input into a graph. **`FAC_adc_get_raw_channel_value()` has no bounds check**: passing an index above 2 reads past the array. The generator must emit only the literal indices `0`, `1`, `2`.

Normalizing the auxiliary analog input is one primitive:

```c
fac_value_t aux = FAC_math_from_range(FAC_adc_get_raw_channel_value(1), 0, FAC_adc_GET_resolution());
```

### 8.4 Receiver — `FAC_Code/fac_std_receiver.h`

```c
uint16_t FAC_std_receiver_GET_channel(uint8_t chNumber);   /* 1-BASED, returns 0 .. 999 */
uint8_t  FAC_std_receiver_GET_is_connected(void);
```

The mix and function frameworks already read the receiver for their configured inputs, and that is the path a graph should normally use — the channel number then stays user-configurable through the FAC Tool. Read a channel directly **only** when the graph needs a channel the framework does not give it (a mix needing a 9th input, a special function needing a second channel).

- **`chNumber` is 1-based.** `1 … RECEIVER_CHANNELS_NUMBER` (8); anything outside returns `0` without touching a backend.
- The returned value is `0 … 999` with the deadzone already applied, and needs `FAC_math_from_range(v, 0, RECEIVER_CHANNEL_RESOLUTION)` to become normalized.
- Each call **triggers a recalculation** from the active receiver backend — it is not a free field read. Do not call it repeatedly for the same channel in one pass; read it once into a local.
- `FAC_std_receiver_GET_is_connected()` becomes `TRUE` once any channel has read non-zero and **never goes back to `FALSE`**. It is the arming gate, not a live link-quality signal — do not build a failsafe out of it.

### 8.5 Application state — `FAC_Code/fac_app.h`

```c
uint8_t FAC_app_GET_current_state(void);   /* FAC_STATE_DISARMED / _NORMAL / _CUTOFF */
uint8_t FAC_app_GET_battery_type(void);
int32_t map_int32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
```

Reading the state from inside a mix is nearly always pointless: the chain only ever runs in `FAC_STATE_NORMAL` ([§ 2](#2-execution-model--what-runs-when-how-often)), so `FAC_app_GET_current_state()` returns `FAC_STATE_NORMAL` every time a graph asks. It is listed here so the tool knows **not** to offer "is armed?" as a block condition. What a graph can legitimately detect is the *gap* left by a disarm — see [§ 9.4](#94-the-disarm-gap).

`map_int32` clamps `x` to `in_max` but **not** to `in_min`, and uses 64-bit intermediates. `map_float` exists but is forbidden here.

### 8.6 Settings — `FAC_Code/fac_settings.h`

```c
uint16_t FAC_settings_GET_value(uint8_t code);   /* returns 0 for code >= FAC_SETTINGS_CODE_LAST */
```

Already included by the boilerplate. A graph may read any setting to make itself user-tunable, but there is no free slot: `enum FAC_SETTINGS_CODE` has **64 entries**, all assigned, and adding one is a firmware change with EEPROM and FAC Tool consequences ([§ 12](#12-compatibility-and-versioning)). A generated graph therefore has to treat its own tuning constants as **compile-time literals baked in by the tool**, not as settings — which is exactly what makes regenerating the file the way to retune it.

Reading a setting on every pass is cheap (an array index plus a bounds check), but the values a mix cares about are already cached by `FAC_mixes_init()` / `FAC_functions_init()` and re-read on *apply*.

---

## 9. State and time

Stateful blocks — latch, edge detector, slew limiter, filter, timer — are what a graph needs for self-righting, ramps and toggles, and they are the part of the contract most easily got wrong. Everything here is at the API level; how the tool represents them in its UI is its own business.

### 9.1 Where state lives

**File-scope `static` variables, declared above the update function**, in the `/* PRIVATE FUNCTIONS AND VARIABLES */` region. Rules:

- They live in `.bss` and are **zero-initialized at boot**. A graph must be correct with all state at `0` on its first pass.
- They are **not touched by `FAC_mixes_init()` / `FAC_functions_init()`**, which only reset the framework's own input/output arrays. So state **survives an *apply settings*** from the FAC Tool — a live re-configure does not reset a latch.
- They persist across a disarm/re-arm too, because nothing clears them. See [§ 9.4](#94-the-disarm-gap).
- Never `volatile` (nothing else touches them; `volatile` only blocks optimization), never `const`, never global — a non-`static` variable at file scope pollutes the link namespace of the whole firmware.
- RAM is 16 KB total. Keep state to a handful of `int32_t`s per graph.
- For a **multi-instance special function**, state must be an array indexed by `sFunctionID` — see [§ 5.2](#52-multiple-instances--the-rule-that-bites).

### 9.2 Patterns and their exact integer form

These are the recipes; each is exact, reproducible in a simulator, and division-free unless noted.

**Previous value / edge detection** — 0 divisions:

```c
static fac_value_t previous = FAC_VALUE_ZERO;
fac_value_t current = inputs[INPUT_X];
uint8_t rising = (previous <= THRESHOLD) && (current > THRESHOLD);
previous = current;                       /* update last, after every use */
```

**Toggle latch with hysteresis** — 0 divisions. Two thresholds, never one, or a stick sitting on the edge chatters at 1 kHz:

```c
static uint8_t latched = FALSE;
static uint8_t armed   = TRUE;
fac_value_t v = inputs[INPUT_SWITCH];
if (armed && v > 600) { latched = !latched; armed = FALSE; }
if (v < 400) { armed = TRUE; }
outputs[OUTPUT_X] = latched ? FAC_VALUE_MAX : FAC_VALUE_MIN;
```

**Slew-rate limiter (ramp)** — 0 divisions, and exact: it always reaches the target:

```c
static fac_value_t current = FAC_VALUE_ZERO;
fac_value_t target = inputs[INPUT_THROTTLE];
const fac_value_t step = 20;              /* units per pass; at ~1 kHz, 0 -> 1000 in ~50 ms */
if      (target > current + step) current += step;
else if (target < current - step) current -= step;
else                             current  = target;
outputs[OUTPUT_MOTOR] = current;
```

> The step is **per pass**, and the pass rate is approximate. If the ramp time has to be a real duration, gate the step on a `HAL_GetTick()` interval ([§ 9.3](#93-time)) instead of assuming 1000 passes per second.

**First-order low-pass filter** — 1 division, and it carries a trap worth stating plainly:

```c
/* WRONG: with truncation toward zero this stalls.
 * When |x - v| < N the quotient is 0 and v never reaches x — a permanent dead band of N units. */
v += (x - v) / N;

/* RIGHT: keep the state on a x1000 scale so the dead band shrinks by the same factor */
static int32_t acc = 0;                                  /* filtered value, x1000 */
const int32_t N = 16;                                    /* time constant, in passes */
acc += ((int32_t) x * 1000 - acc) / N;                   /* 1 division */
fac_value_t out = acc / 1000;                            /* 1 division */
```

The residual dead band is now `N/1000` of a normalized unit instead of `N` units. Make `N` a power of two and both divisions fold to shifts, taking the cost to zero. **A simulator must reproduce the truncation exactly** — `Math.trunc`, not `Math.floor`, and negative numerators are where the two differ.

**Integrator** — accumulate on `int32_t` (group 2, no saturation) and clamp explicitly with `FAC_math_clamp_to`, never let it grow unbounded. Anything integrating a gyro rate needs an anti-windup bound and a reset condition; a graph with an unbounded integrator will eventually overflow, which is undefined behaviour.

### 9.3 Time

```c
uint32_t HAL_GetTick(void);   /* free-running millisecond counter, from stm32f0xx_hal.h */
```

- Unit is **1 ms**. Available everywhere, no include needed beyond what the boilerplate already pulls in.
- It wraps every 2³² ms ≈ **49.7 days**. **Always compare differences, never absolute values**: `if ((uint32_t)(HAL_GetTick() - t0) >= period)` is correct across the wrap; `if (HAL_GetTick() > t0 + period)` is not.
- The pass rate is **approximately** 1 kHz, not guaranteed. Anything that must be a real duration has to be measured with the tick, not counted in passes.

Caching an expensive read (the battery being the canonical case, [§ 8.2](#82-battery--fac_codefac_batteryh)):

```c
static uint32_t lastRead = 0;
static uint16_t cachedMv = 0;
uint32_t now = HAL_GetTick();
if ((uint32_t)(now - lastRead) >= 200) {  /* 5 Hz is plenty for a voltage */
	cachedMv = FAC_battery_GET_voltage();
	lastRead = now;
}
```

### 9.4 The disarm gap

This is the trap that only shows on hardware. Because the chain runs **only in `FAC_STATE_NORMAL`**, a disarm freezes every `static` in every generated file. On re-arm the graph resumes with:

- a latch still holding whatever it held before the disarm (a weapon toggle stays *on*);
- a slew limiter still at its last ramped value, so the first pass after re-arm jumps by one `step` from a value that no longer reflects the sticks;
- a filter holding a sample from an arbitrarily long time ago;
- a timer whose `t0` is now far in the past, so **every pending interval fires at once** on the first pass back.

The fix is a staleness guard the generator should emit for any graph holding state. One extra `static` covers all of it:

```c
static uint32_t lastRun = 0;
uint32_t now = HAL_GetTick();
if ((uint32_t)(now - lastRun) > 100) {    /* > 100 ms since the last pass: we were not running */
	/* reset the graph state to its boot values here */
}
lastRun = now;
```

`100` ms is two orders of magnitude above the nominal pass period and well below any real disarm, so it separates the two cleanly. **Whether a given block resets or holds is a design choice** — a weapon latch arguably *should* reset to off, a trim value arguably should not — but it must be a choice the tool makes explicitly, not an accident of what `static` happens to do.

---

## 10. Hard rules for generated code

Every one of these is enforced by the platform, the build flags, or the safety model. A generator that violates them produces code that does not compile, does not fit the loop budget, or resets the board.

| # | Rule | Why |
|---|---|---|
| 1 | **No `float`, no `double`, no `math.h`** | No FPU. Every float operation is a soft-float library call of hundreds of cycles. The whole mix/function/mapper chain is verified to contain **zero soft-float calls** — keep it that way |
| 2 | **No blocking** — no `HAL_Delay`, no wait loop, no busy poll | The IWDG is refreshed once per pass, after the mapper returns. A block longer than ~400 ms resets the board |
| 3 | **No dynamic allocation** — no `malloc`, no VLAs | 16 KB of RAM, no heap in this firmware |
| 4 | **No recursion** | Bounded stack, no analysis to prove depth |
| 5 | **No I²C, no EEPROM, no USB, no direct register access** | The one exception is `FAC_IMU_update()`, whose guards are documented in [§ 8.1](#81-imu--fac_codefac_imuh) |
| 6 | **No device writes** — motors, servos, LED, buzzer | They belong to the mapper and the state machine ([§ 6.3](#63-what-generated-code-must-not-call)) |
| 7 | **No setting writes** | Settings are the FAC Tool's domain; a write from a mix would fight the tool and the EEPROM |
| 8 | **`-Wall` clean** | An unused `static` function or variable is a real warning in the build log. Keep `__attribute__((unused))` on the `mix_id` / `first_special_function_id` marker; do not emit unused locals |
| 9 | **Every intermediate fits in `int32_t`** | Signed overflow is undefined behaviour, and a JavaScript simulator would not reproduce it. Validate the range constraints of [§ 7](#7-math-api--fac_mathh) when the graph is built, not when it runs |
| 10 | **Never divide by a value that can be zero** | `FAC_math_mul_scaled` / `_div_scaled` guard it and return `0`; a bare `/` in generated code does not |
| 11 | **The boilerplate is verbatim** | Input fetch, the zero-init of `outputs[]`, the final clamp loop, the write-back. Generated code lives strictly between the `INSERT YOUR CODE HERE` markers plus the file-scope statics |
| 12 | **Comments and identifiers in English**, tabs for indentation, K&R braces | Project convention. Match the existing non-native spellings where they are part of the API (`SPECIAL_FUNCITONS_NUMBER`, `special_functions_outouts`, `FAC_std_reciever_init`) — do not "fix" them |

---

## 11. Registration — the part a file generator cannot do

Emitting the two files is the easy half. A new mix or special function also needs edits in **other** files, which dropping files into a folder cannot perform. This is the known open problem, and the tool has to solve it one of the two ways below.

### 11.1 The manual procedure today

**For a mix** — three edits outside the new files:

| # | File | Edit |
|---|---|---|
| 1 | `Core/Inc/FAC_Code/mixes_functions/fac_mixes.h` | Add `FAC_MIX_<NAME>,` to `enum FAC_MIXES_ID`, **before `FAC_MIX_LAST`** |
| 2 | `Core/Src/FAC_Code/mixes_functions/fac_mixes.c` | Add `#include "FAC_Code/mixes_functions/mixes/fac_<name>_mix.h"` in the `/* NEW MEXES */` block |
| 3 | `Core/Src/FAC_Code/mixes_functions/fac_mixes.c` | Add `case FAC_MIX_<NAME>: FAC_<name>_mix_update(); break;` to `FAC_mix_update()` |

The settings side is automatic: `FAC_SETTINGS_CODE_ACTIVE_MIX` has `max = FAC_MIX_LAST-1` in `settings[]`, so it follows the enum.

**For a special function** — the same three, plus one that is easy to miss:

| # | File | Edit |
|---|---|---|
| 1 | `Core/Inc/FAC_Code/mixes_functions/fac_functions.h` | Add the ID(s) to `enum FAC_SPECIAL_FUNCTIONS_ID`, before `FAC_SPECIAL_FUNCTION_LAST`. **`FAC_SPECIAL_FUNCTION_LAST` must stay ≤ 20** (`SPECIAL_FUNCITONS_NUMBER`) |
| 2 | `Core/Src/FAC_Code/mixes_functions/fac_functions.c` | Add the `#include` |
| 3 | `Core/Src/FAC_Code/mixes_functions/fac_functions.c` | Add the `case` label(s) to `FAC_functions_update()` — grouped without `break` for a multi-instance function, `break` only on the last |
| 4 | `Core/Src/FAC_Code/fac_settings.c` | ⚠ **The five mapper rows have a hardcoded `max` of `200+10`.** Today `FAC_SPECIAL_FUNCTION_LAST` is 11, so `200+10` reaches exactly the last ID. **A 12th special function is unreachable by the mapper until those five `max` values are raised.** Nothing warns about this — the function compiles, registers, runs, and simply cannot be linked to a device |

**And in both cases**: adding a new `.c` file requires **regenerating the STM32CubeIDE build files** (`Debug/**/subdir.mk`), from the IDE, or the file is silently not compiled. `Debug/` and `Release/` are git-ignored and contain absolute paths; never hand-edit them.

### 11.2 What the tool should emit today

Given the above, a generator that only writes two files leaves the user with four to six manual edits. The minimum useful output is therefore **the two files plus a registration report**: the exact snippets and their insertion points, ideally as a unified diff the user can apply, and an explicit reminder to refresh the project in STM32CubeIDE. That is enough to make the tool useful before any firmware refactor happens.

### 11.3 The table-driven target

The cleaner target — worth keeping in mind whenever `fac_mixes.c` / `fac_functions.c` are touched — is to **replace the `switch` dispatch with a table**, so that registering a mix becomes adding *one row* instead of editing three sites.

Sketch for mixes:

```c
/* fac_mixes.h */
typedef void (*fac_mix_update_fn)(void);

typedef struct MixDescriptor {
	uint8_t           id;        /* the FAC_MIXES_ID value, persisted in settings — see below */
	fac_mix_update_fn update;    /* the mix's update function */
	const char       *name;      /* optional, for the FAC Tool / debugging */
} MixDescriptor;

/* fac_mixes.c */
static const MixDescriptor mixTable[] = {
	{ FAC_MIX_NONE,        NULL,                     "none"        },
	{ FAC_MIX_SIMPLE_TANK, FAC_simple_tank_mix_update, "simple tank" },
	/* one row per mix */
};

void FAC_mix_update(void) {
	uint8_t current = FAC_mixes_GET_current_mix();
	for (int i = 0; i < (int)(sizeof(mixTable) / sizeof(mixTable[0])); i++) {
		if (mixTable[i].id == current) {
			if (mixTable[i].update) mixTable[i].update();
			return;
		}
	}
}
```

Four properties this has to keep:

1. **`FAC_SETTINGS_CODE_ACTIVE_MIX`'s `max` must still track the number of mixes.** With a table it can no longer be `FAC_MIX_LAST-1` unless the enum is kept in step; either keep both (enum for the IDs, table for the dispatch) or derive the bound from the table size at init.
2. **IDs are persisted.** `ACTIVE_MIX` and the mapper links live in EEPROM and are addressed by the FAC Tool, so IDs are **append-only** — never reorder, never reuse a retired number. The table makes reordering *look* safe, which is precisely the new hazard it introduces; a comment at the table saying so is not optional.
3. The function-pointer indirection costs one extra load per call, on a path that runs once per pass. Irrelevant against the mapper's own cost.
4. `NULL` must be handled — `FAC_MIX_NONE` has no update function today, and the `switch` handles that by having an empty `case`.

The same shape works for special functions, with an extra field for the **instance count** so one descriptor covers a multi-instance function and the dispatcher can compute the slot from the ID.

**Even this still leaves one file to edit.** The step that would let a generator own registration completely is to move the table rows into a **generated list header** — an X-macro file such as `fac_mixes_list.h` containing nothing but one macro invocation per mix, which the tool rewrites wholesale from the set of files present in `mixes/`. Then adding a mix is: drop two files, regenerate one list file, refresh the IDE project. That is the smallest reachable manual step, since the CubeIDE makefile regeneration cannot be automated from a browser anyway.

---

## 12. Compatibility and versioning

Things a generated artifact can silently break, and the constraint that prevents each:

| Constraint | Consequence of breaking it |
|---|---|
| **`enum FAC_MIXES_ID` is append-only** | The active mix is stored in EEPROM as a number. Reordering makes an existing configuration select a different mix on the next boot |
| **`enum FAC_SPECIAL_FUNCTIONS_ID` is append-only** | The mapper links store `200 + index`. Inserting an ID in the middle silently re-points every mapped device to a different function |
| **`FAC_SPECIAL_FUNCTION_LAST` ≤ `SPECIAL_FUNCITONS_NUMBER` (20)** | The framework arrays are 20 entries; a 21st ID reads and writes past them |
| **The five mapper `max` values are `200+10` today** | A special function with index > 10 cannot be linked to any device until they are raised ([§ 11.1](#111-the-manual-procedure-today)) |
| **Setting codes are FAC Tool ABI and the EEPROM address is `code*2`** | Never insert a setting in the middle; append before `FAC_SETTINGS_CODE_LAST` only |
| **Bumping the firmware version resets user settings** | `FIRMWARE_VERSION_TAG` is a hash of MAJOR/MINOR/PATCH; a mismatch with the EEPROM marker rewrites **all defaults**. Intended when the settings layout changes, but it happens on *any* version bump — 10 fast boot blinks mean defaults were written, 3 mean a normal load |
| **`Debug/**/subdir.mk` is generated and holds absolute paths** | A new `.c` not registered there is silently not compiled — the mix appears in the menu and does nothing |

---

## 13. Conformance checklist

What to verify on a generated pair before handing it to a build. A tool that checks these mechanically will catch essentially every class of defect described above.

**Structure**

- [ ] `.c` and `.h` named `fac_<name>_mix.{c,h}` / `fac_<name>_function.{c,h}`, header guard unique and derived from the file name
- [ ] The boilerplate is byte-identical to the template outside the marked regions
- [ ] Update function declared `(void)` (mix) or `(uint8_t sFunctionID)` (function), matching in `.c`, `.h` and the dispatcher
- [ ] `mix_id` / `first_special_function_id` present, `static const uint8_t`, `__attribute__((unused))` kept
- [ ] `INPUT_*` / `OUTPUT_*` defines present for every used index, numbers not renumbered
- [ ] A real `DESCRIPTION` block, in English
- [ ] Every identifier used has its header included — in particular `main.h` for `TRUE` / `FALSE` and `stdlib.h` for `abs()`, neither of which the boilerplate pulls in

**Correctness**

- [ ] No `float`, `double`, `math.h`, `malloc`, `HAL_Delay`, blocking loop, recursion, VLA
- [ ] No call to a motor, servo, settings-write, EEPROM or USB function
- [ ] Every `FAC_math_*` call respects its documented range constraint ([§ 7](#7-math-api--fac_mathh))
- [ ] Every intermediate proven to fit `int32_t` over the full input domain
- [ ] No division by a possibly-zero value outside the guarded primitives
- [ ] All 10 outputs written or left at `FAC_VALUE_ZERO` (the boilerplate's zero-init covers the unused ones)

**IMU, if used**

- [ ] `FAC_IMU_update()` called once, at the top of the generated region
- [ ] `FAC_IMU_GET_status() != HAL_ERROR` guard present, with a defined fallback

**State, if used**

- [ ] All state is file-scope `static`, correct when zero-initialized
- [ ] Multi-instance special function: state is an array indexed by `sFunctionID`
- [ ] Time comparisons use `(uint32_t)(now - then) >= period`, never `now > then + period`
- [ ] A staleness guard covers the disarm gap ([§ 9.4](#94-the-disarm-gap)), with an explicit reset-or-hold decision per block
- [ ] Filters keep their state on a scaled accumulator, not the naive `v += (x-v)/N`

**Simulation parity**

- [ ] Every division in the simulator is `Math.trunc`, never `Math.floor`
- [ ] Group-1 primitives clamp arguments *and* result, in the order `fac_math.h` does
- [ ] Group-2 primitives do **not** clamp
- [ ] The guarded returns (`scale == 0` → 0, `b == 0` → 0, `in_max == in_min` → 0, `atan2(0,0)` → 0, `sqrt(v <= 0)` → 0) are reproduced
- [ ] `sqrt` is reproduced on **unsigned** 32-bit arithmetic (`>>>` in JavaScript, never a signed shift), and its argument is range-checked against `int32_t` *after* any scale pre-multiplication

**Registration**

- [ ] Enum entry emitted, appended before `_LAST`, never inserted
- [ ] Dispatcher `case` and `#include` emitted
- [ ] Special function: `FAC_SPECIAL_FUNCTION_LAST` still ≤ 20, and mapper `max` raised if the new index exceeds 10
- [ ] The user is told to refresh the STM32CubeIDE project so `subdir.mk` picks up the new `.c`

---

*© The Floppy Lab™ — F.A.C. V2 firmware.*
