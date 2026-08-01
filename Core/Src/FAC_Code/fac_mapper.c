/*
 * fac_mapper.c
 *
 *	HOW MAPPER WORKS:
 *	The mapper reads from the settings which device is linked
 *	to which output of the active mix or special function, and sets
 *	the values ​​of the various outputs to the corresponding devices.
 *	Mix outputs use the prefix 100 to identify them, while special
 *	functions use the prefix 200. (e.g., m1 = 102, motor 1 takes
 *	the value of output 2 of the active mix.)
 *	The mapper takes the active outputs from the active mixes or
 *	functions and conditions the value of the given output
 *	(in standard format) to be understandable by the devices.
 *	If the special function has input channel = 0, it is disabled
 *	and should not be calculated.
 *
 *	The mapper calculates the outputs at the time of the update call (FAC_mapper_apply_to_devices()),
 *	and for each peripheral (if active) it conditions the outputs
 *	and applies them to the peripheral.
 *
 *
 *  Created on: Sep 5, 2025
 *      Author: filippo-castellan
 */

#include "FAC_Code/fac_mapper.h"
#include "FAC_Code/mixes_functions/fac_mixes.h"
#include "FAC_Code/mixes_functions/fac_functions.h"
#include "main.h"

#include "FAC_Code/fac_settings.h"
#include "FAC_Code/fac_motors.h"
#include "FAC_Code/fac_servo.h"
#include "FAC_Code/config.h"	// explicit, the debug switches below must not depend on another header

#ifdef DEBUG_UTILS
#include "FAC_Code/fac_debug_utils.h"

#ifdef CRONOMETER_FAC_MAPPER
/* EXECUTION TIMES OF THE MAPPER BLOCKS, IN MICROSECONDS (0 MEANS OVERFLOW, SEE FAC_debug_utils_crono_stop) */
// they are volatile only to keep the optimizer from removing them: nothing reads them, they are meant
// to be watched from the debugger while the robot is running
static volatile uint16_t links_read_execution_time = 0;	// the five settings reads of the links
static volatile uint16_t mix_update_execution_time = 0;	// the single FAC_mix_update call of the active mix
static volatile uint16_t functions_update_execution_time = 0;// sum of every FAC_functions_update call of this pass
static volatile uint16_t functions_update_calls = 0;	// how many times FAC_functions_update ran, NOT a time
static volatile uint16_t mix_functions_update_execution_time = 0;// mix + functions, the loop around them is not measured
static volatile uint16_t m1_apply_execution_time = 0;	// per device, each dc motor rewrites two soft pwm buffers
static volatile uint16_t m2_apply_execution_time = 0;
static volatile uint16_t m3_apply_execution_time = 0;
static volatile uint16_t s1_apply_execution_time = 0;	// servos only write a timer register
static volatile uint16_t s2_apply_execution_time = 0;
static volatile uint16_t devices_apply_execution_time = 0;// sum of the five devices above
static volatile uint16_t mapper_total_execution_time = 0;// sum of all the blocks of this function
#endif
#endif

/*
 * @brief	According to the output value of the linked output apply the DC motor speed and direction
 * @note	Link value is the settings value of the mapper not the actual output value
 * @note	Only 100..109 and 200..210 are meaningful, but the settings range is a single 0..210
 * 			interval and cannot exclude the gap, so an invalid link resolves to 0.0f in the getters
 */
static void FAC_mapper_apply_to_DCmotor(uint8_t motorNumber, uint8_t linkValue) {
	float outputLinkedValue = 0.0f;

	/* EXTRACT THE CORRECT OUTPUT VALUE */
	if (linkValue / 100 == 1) {	// linked to a mix output	(100 prefix)
		outputLinkedValue = FAC_mixes_GET_output(linkValue % 100); // extract the output value linked
	} else if (linkValue / 100 == 2) {	// linked to a special function output	(200 prefix)
		outputLinkedValue = FAC_functions_GET_output(linkValue % 200); // extract the output value linked
	}

	/* CALCULATE SPEED AND DIRECTION */
	uint8_t dir = FORWARD;	// before the absolute calculate the direction
	if (outputLinkedValue < 0.0f) {	// already set to forward, so I need to check if it is backward
		dir = BACKWARD;
	}
	if (outputLinkedValue < 0.0f)
		outputLinkedValue = outputLinkedValue * (-1);
	uint16_t speed = (uint16_t) (outputLinkedValue * MOTOR_SPEED_RESOLUTION); // translate the output value to understandable value for servos and motors

	/* APPLY TO THE MOTOR */
	FAC_motor_set_speed_direction(motorNumber, dir, speed);
}

/*
 * @brief	According to the output value of the linked output apply the servo
 * @note	Link value is the settings value of the mapper not the actual output value
 * @note	Only 100..109 and 200..210 are meaningful, but the settings range is a single 0..210
 * 			interval and cannot exclude the gap, so an invalid link resolves to 0.0f in the getters
 */
static void FAC_mapper_apply_to_servo(uint8_t servoNumber, uint8_t linkValue) {
	float outputLinkedValue = 0.0f;

	/* EXTRACT THE CORRECT OUTPUT VALUE */
	if (linkValue / 100 == 1) {	// linked to a mix output	(100 prefix)
		outputLinkedValue = FAC_mixes_GET_output(linkValue % 100); // extract the output value linked
	} else if (linkValue / 100 == 2) {	// linked to a special function output	(200 prefix)
		outputLinkedValue = FAC_functions_GET_output(linkValue % 200); // extract the output value linked
	}

	/* CALCULATE SERVO POSITION/ESC VELOCITY */
	uint16_t position = (uint16_t) (map_float(outputLinkedValue, -1.0f, 1.0f, 0.0f, (float) SERVO_POSITION_RESOLUTION)); // translate the output value to understandable value for servos and motors

	/* APPLY TO THE MOTOR */
	FAC_servo_set_position(servoNumber, position);
}

/*
 * @brief		This function apply all outputs to the linked device such as DC motor and servo
 * @IMPORTANT	!! THIS FUNCTION MUST BE CALLED EVERY LOOP TO MANTAIN THE DEVICES STATUS UPDATED !!
 * @note		If a motor/servo is not mapped to any output it will be disabled (DC motors are setted to 0 velocity, breaked, servo pwm is disabled)
 */
void FAC_mapper_apply_to_devices(void) {
	/*
	 *	The link value can be:
	 *	-	100+i where 100 is the prefix indicating that is a mix output, and 'i' is the output number of the mix (0 to max mix output)
	 *	-	200+i where 200 is the prefix indicating that is a special function output, and 'i' indicates the output number (0 to max functions number)
	 */
#ifdef CRONOMETER_FAC_MAPPER
	/* the measures are accumulated on locals and published on the volatiles only at the end of the
	 * function: this way the debugger never reads a variable while it is being cleared or summed */
	uint16_t tLinks = 0, tMix = 0, tFunctions = 0, tCalls = 0;
	uint16_t tM1 = 0, tM2 = 0, tM3 = 0, tS1 = 0, tS2 = 0;// a device that is not mapped stays at 0
	FAC_debug_utils_crono_start();
#endif
	uint8_t m1_link = FAC_settings_GET_value(FAC_SETTINGS_CODE_MAPPER_M1);
	uint8_t m2_link = FAC_settings_GET_value(FAC_SETTINGS_CODE_MAPPER_M2);
	uint8_t m3_link = FAC_settings_GET_value(FAC_SETTINGS_CODE_MAPPER_M3);
	uint8_t s1_link = FAC_settings_GET_value(FAC_SETTINGS_CODE_MAPPER_S1);
	uint8_t s2_link = FAC_settings_GET_value(FAC_SETTINGS_CODE_MAPPER_S2);

	uint8_t links[5] = {
		m1_link,
		m2_link,
		m3_link,
		s1_link,
		s2_link };

#ifdef CRONOMETER_FAC_MAPPER
	tLinks = FAC_debug_utils_crono_stop();
#endif

	/* CHECK IF MIX AND IF/WHICH SPECIAL FUNCTION MUST BE CALCULATED */
	uint8_t mixUpdated = FALSE;
	uint8_t functionsUpdated[SPECIAL_FUNCITONS_NUMBER];
	for (int i = 0; i < SPECIAL_FUNCITONS_NUMBER; i++)	// initialize to not updated the whole array
		functionsUpdated[i] = FALSE;

	/* UPDATE ALL OUTPUTS OF THE MIX AND ACTIVE SPECIAL FUNCTIONS */
	for (int i = 0; i < sizeof(links) / sizeof(links[0]); i++) {	// check and update the active functions/mix
		if (links[i] / 100 == 1 && !mixUpdated) {	// if the device is linked to the mix and it is not already been calculated
#ifdef CRONOMETER_FAC_MAPPER
			FAC_debug_utils_crono_start();
#endif
			FAC_mix_update();	// update the outputs values of the mix/* DC MOTOR 1*/
#ifdef CRONOMETER_FAC_MAPPER
			tMix = FAC_debug_utils_crono_stop();
#endif
			mixUpdated = TRUE;	// to not calculate it again
		}

		if (links[i] / 100 == 2) {	// 200+0 is function at position 0 (see FAC_SPECIAL_FUNCTIONS_ID for number reference)
			uint8_t linkedFunctionNumber = links[i]%200;
			if (linkedFunctionNumber < SPECIAL_FUNCITONS_NUMBER) {	// the settings range cannot exclude the invalid link values, so check here
				if(!functionsUpdated[linkedFunctionNumber])
#ifdef CRONOMETER_FAC_MAPPER
				{
					FAC_debug_utils_crono_start();
					FAC_functions_update(linkedFunctionNumber);
					tFunctions += FAC_debug_utils_crono_stop();
					tCalls++;
				}
#else
				FAC_functions_update(linkedFunctionNumber);
#endif
				functionsUpdated[linkedFunctionNumber] = TRUE;
			}
		}
	}


	/* TRANSFER THE OUTPUTS TO THE CORRECT DEVICES */
	/* DC MOTOR 1 */
	if (m1_link) {	// ( mN_link == 0 -> not used) if used apply settings to it
#ifdef CRONOMETER_FAC_MAPPER
		FAC_debug_utils_crono_start();
#endif
		FAC_mapper_apply_to_DCmotor(1, m1_link);
#ifdef CRONOMETER_FAC_MAPPER
		tM1 = FAC_debug_utils_crono_stop();
#endif
	}
	/* DC MOTOR 2 */
	if (m2_link) {	// ( mN_link == 0 -> not used) if used apply settings to it
#ifdef CRONOMETER_FAC_MAPPER
		FAC_debug_utils_crono_start();
#endif
		FAC_mapper_apply_to_DCmotor(2, m2_link);
#ifdef CRONOMETER_FAC_MAPPER
		tM2 = FAC_debug_utils_crono_stop();
#endif
	}
	/* DC MOTOR 3 */
	if (m3_link) {	// ( mN_link == 0 -> not used) if used apply settings to it
#ifdef CRONOMETER_FAC_MAPPER
		FAC_debug_utils_crono_start();
#endif
		FAC_mapper_apply_to_DCmotor(3, m3_link);
#ifdef CRONOMETER_FAC_MAPPER
		tM3 = FAC_debug_utils_crono_stop();
#endif
	}
	/* SERVO 1 */
#ifdef CRONOMETER_FAC_MAPPER
	FAC_debug_utils_crono_start();
#endif
	if (s1_link) {	// ( sN_link == 0 -> not used) if used apply settings to it
		FAC_servo_enable(1);
		FAC_mapper_apply_to_servo(1, s1_link);
	} else
		FAC_servo_disable(1);
#ifdef CRONOMETER_FAC_MAPPER
	tS1 = FAC_debug_utils_crono_stop();
#endif
	/* SERVO 2 */
#ifdef CRONOMETER_FAC_MAPPER
	FAC_debug_utils_crono_start();
#endif
	if (s2_link) {	// ( sN_link == 0 -> not used) if used apply settings to it
		FAC_servo_enable(2);
		FAC_mapper_apply_to_servo(2, s2_link);
	} else
		FAC_servo_disable(2);
#ifdef CRONOMETER_FAC_MAPPER
	tS2 = FAC_debug_utils_crono_stop();

	/* PUBLISH THE MEASURES */
	/* the cronometer has a single counter, so the whole function cannot be measured while its blocks
	 * are, the totals are the sum of the parts (the start/stop overhead is counted in every part) */
	links_read_execution_time = tLinks;
	mix_update_execution_time = tMix;
	functions_update_execution_time = tFunctions;
	functions_update_calls = tCalls;
	mix_functions_update_execution_time = tMix + tFunctions;// the loop around them is too cheap to measure
	m1_apply_execution_time = tM1;
	m2_apply_execution_time = tM2;
	m3_apply_execution_time = tM3;
	s1_apply_execution_time = tS1;
	s2_apply_execution_time = tS2;
	devices_apply_execution_time = tM1 + tM2 + tM3 + tS1 + tS2;
	mapper_total_execution_time = tLinks + tMix + tFunctions
			+ devices_apply_execution_time;
#endif
}
