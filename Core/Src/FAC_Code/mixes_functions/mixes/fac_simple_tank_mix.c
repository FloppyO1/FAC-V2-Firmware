/*
 * fac_simple_tank_mix.c	// 1) of HOW TO MAKE A MIX
 *
 *	HOW TO MAKE A MIX:
 *	1) Copy this file and the fac_template_mix.h.template and rename
 *		with the name of the new mix (ex: fac_simple_tank_mix.c/h)
 *		In the .h files, update the #ifndef and #define with the new name.
 *	2) Fix the include to match the new name you have choose
 *	3) Add to the FAC_MIXES_ID on fac_mixes.h file the new FAC_MIX
 *		(ex: FAC_MIX_SIMPLE_TANK) and set it to the mix_id
 *	4) Rename the FAC_<name>_mix_update() function on this file and
 *		on the fac_name_mix.h
 *	5) Add the FAC_<name>_mix_update() function into a new case into
 *		the fac_mixes.c file inside the FAC_mix_update() function using
 *		as case value your mix_id value.
 *	6) Add the include header file of your new mix inside the
 *		fac_mixes.c (#include "FAC_Code/mixes_functions/mixes/fac_<name>_mix.h")
 *	7) Make a description of the mix and describe all inputs/outputs it takes/gives
 *		Remeber: an input is a remote controller channel converted in
 *		standard values (the channel is chosen by the settings).
 *		If for example you need the gyroscope data you have to manage it
 *		inside your code.
 *	8) Include all the modules you need for your mix logic
 *		(uncomment the modules you need in CUSTOM MIX INCLUDE part)
 *	9) Write your mix logic code
 *
 *
 *  Created on: Sep 5, 2025
 *      Author: filippo-castellan
 */

#include "FAC_Code/mixes_functions/mixes/fac_simple_tank_mix.h"			// 2) of HOW TO MAKE A MIX

/* DEFAULT INCLUDE */ // DON´T TOUCH THIS CODE!!
#include "FAC_Code/mixes_functions/fac_mixes.h"
#include "FAC_Code/fac_settings.h"

/* CUSTOM INCLUDE */								// 8) of HOW TO MAKE A MIX
//#include "FAC_Code/fac_adc.h"
//#include "FAC_Code/fac_fac_imu.h"		// if get_status == HAL_ERROR NOT USE data!!
#include "stdlib.h"
/* PRIVATE FUNCTIONS AND VARIABLES */
// marker only, it records which ID this file implements (see step 5): unused on purpose
static const uint8_t __attribute__((unused)) mix_id = FAC_MIX_SIMPLE_TANK;// 3) of HOW TO MAKE A MIX		(only to know witch mix is this)

/* WHAT THIS MIX DO */								// 7) of HOW TO MAKE A MIX
/*
 * DESCRIPTION:
 * this mix makes the wheels to spin as a tank tracks, so implement a differential steering
 *
 * INPUTs DESCRIPTION:
 * Uncomment the input you want to use and rename it
 * (ex: #define INPUT_THROTTLE 0, or #define INPUT_STEERING 1)
 * !! DONT'T CHANGE THE NUMBER ASIGNED !!
 * Use the INPUT_<NAME> to know the position of where to get the input value in the inputs array
 */
#define INPUT_THROTTLE 0
#define INPUT_STEERING 1
//#define INPUT_NOT_USED 2
//#define INPUT_NOT_USED 3
//#define INPUT_NOT_USED 4
//#define INPUT_NOT_USED 5
//#define INPUT_NOT_USED 6
//#define INPUT_NOT_USED 7

/* OUTPUTs DESCRIPTION:
 * Uncomment the output you want to use and rename it
 * (ex: #define OUTPUT_MOTOR_LEFT 0, or #define OUTPUT_MOTOR_RIGHT 1)
 * !! DONT'T CHANGE THE NUMBER ASIGNED !!
 * Use the OUTPUT_<NAME> to know the position of where to put the output value in the outputs array
 *
 * !! REMEMBER:
 * 		- All outputs must be in a standard format, values from FAC_VALUE_MIN to FAC_VALUE_MAX (-1000 to +1000) !!
 * 			write here what each mix's output is (ex: 0) motor left, 1) motor right 2)not used ...)
 * 		- If an output is for a DC motor, a positive number is considered as forward movement, a negative number is considered as backwards movement of the DC motor
 * 		- If an output is for a Servo vomor/esc, -1000 is considered as 0 and +1000 is considered as 100% (in degrees for servos 0°-180°)
 */
#define OUTPUT_MOTOR_LEFT 0
#define OUTPUT_MOTOR_RIGHT 1
//#define OUTPUT_NOT_USED 2
//#define OUTPUT_NOT_USED 3
//#define OUTPUT_NOT_USED 4
//#define OUTPUT_NOT_USED 5
//#define OUTPUT_NOT_USED 6
//#define OUTPUT_NOT_USED 7
//#define OUTPUT_NOT_USED 8
//#define OUTPUT_NOT_USED 9

/*
 * @brief	Calculate the mix output values
 *
 */
void FAC_simple_tank_mix_update(void) {					// 4) of HOW TO MAKE A MIX
	// this code must be left as it is, DON'T TOUCH IT!
	fac_value_t outputs[MIXES_MAX_OUTPUTS_NUMBER];
	fac_value_t inputs[MIXES_MAX_INPUTS_NUMBER];
	FAC_mixes_update_mix_inputs();// update the mix input in base of the settings and rx channels
	for (int i = 0; i < MIXES_MAX_OUTPUTS_NUMBER; i++) {
		outputs[i] = FAC_VALUE_ZERO;
	}
	for (int i = 0; i < MIXES_MAX_INPUTS_NUMBER; i++) {
		inputs[i] = FAC_mixes_GET_input(i);
	}
	/* INSERT YOUR CODE HERE -START- */				// 9) of HOW TO MAKE A MIX
	/* REMEMBER
	 * - inputs array contains all values of the channel requested for this mix
	 * - in the outputs array you have to write in the same order you written above all outputs for servos and motors
	 * 		outputs values must stay in this range [FAC_VALUE_MIN, FAC_VALUE_MAX], that is [-1000, +1000]
	 * - use the FAC_math_* primitives of fac_math.h, NEVER a float: this mcu has no fpu and every
	 * 		float operation is a library call of hundreds of clock cycles
	 */
	// write here the code of your mix
	/* WHY THE diff TERM EXISTS - DO NOT REPLACE THIS WITH A PLAIN SATURATED SUM !!
	 * A transmitter gimbal moves inside a SQUARE gate, not a circular one: throttle and steering
	 * can both be at their end of travel at the same time, in the corners of the gate.
	 * A plain "left = throttle + steering" clipped at full scale would behave as if the gate were
	 * circular, and every corner of the square would saturate to the same value, losing the
	 * difference between them. The diff term redistributes the leftover travel instead, so the
	 * corners of the gate stay distinguishable and the robot keeps steering at full throttle. */
	fac_value_t inThrottle = inputs[INPUT_THROTTLE];	// already normalized, [-1000, +1000]
	fac_value_t inSteering = inputs[INPUT_STEERING];

	/* the sums are deliberately NOT saturated here: they are allowed to reach twice the full
	 * scale, and the /2 below brings them back. Saturating early is what would collapse the
	 * corners of the square gate, see the note above */
	int32_t left = inThrottle + inSteering;
	int32_t right = inThrottle - inSteering;
	int32_t diff = FAC_math_abs(inThrottle) - FAC_math_abs(inSteering);

	if (left < 0)
		left = left - abs(diff);
	else
		left = left + abs(diff);

	if (right < 0)
		right = right - abs(diff);
	else
		right = right + abs(diff);

	outputs[OUTPUT_MOTOR_LEFT] = left / 2;	// /2 because with the mix the range became twice the max value
	outputs[OUTPUT_MOTOR_RIGHT] = right / 2;

	/* INSERT YOUR CODE HERE -END- */
	// keep outputs in range
	for (int i = 0; i < MIXES_MAX_OUTPUTS_NUMBER; i++) {
		outputs[i] = FAC_math_clamp(outputs[i]);
	}
	// update outputs values on mixes struct
	FAC_mixes_update_mix_outputs(outputs);
}
