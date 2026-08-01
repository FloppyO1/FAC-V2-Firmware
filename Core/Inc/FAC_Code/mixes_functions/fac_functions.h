/*https://github.com/FloppyO1/Floppy-Ant-Controller/tree/FAC_V2/SOFTWARE/FAC%20firmware/V2/FAC%20Firmware%20V2
 * fac_functions.h
 *
 *  Created on: Sep 6, 2025
 *      Author: filippo-castellan
 */

#ifndef INC_FAC_CODE_MIXES_FUNCTIONS_FAC_FUNCTIONS_H_
#define INC_FAC_CODE_MIXES_FUNCTIONS_FAC_FUNCTIONS_H_

#include "stm32f0xx_hal.h"
#include "FAC_Code/config.h"
#include "FAC_Code/mixes_functions/fac_math.h"	// fac_value_t and the math primitives a function is built from

#define SPECIAL_FUNCITONS_NUMBER 20

typedef struct SpecialFunctions {
	uint8_t special_functions_input_channels[SPECIAL_FUNCITONS_NUMBER];	// 20 possible function, each function has its own input channel
	fac_value_t special_functions_inputs[SPECIAL_FUNCITONS_NUMBER];		// 20 normalized inputs one for each function [-1000, +1000]
	fac_value_t special_functions_outouts[SPECIAL_FUNCITONS_NUMBER];		// 20 normalized outputs one for each function [-1000, +1000]
} SpecialFunctions;

enum FAC_SPECIAL_FUNCTIONS_ID {		// 8) of HOW TO MAKE A SPECIAL FUNCTION
	// This enum defines the position of the inputs and outputs inside the arrays
	/* MAX SUPPORTED FUNCTIONS ARE 20!!
	 * FAC_SPECIAL_FUNCTION_LAST must be the number 20 at maximum
	 */
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_1ST,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_2DN,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_3RD,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_4TH,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_5TH,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_6TH,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_7TH,
	FAC_SPECIAL_FUNCTION_DIRECT_LINK_TO_CHANNEL_8TH,

	FAC_SPECIAL_FUNCTION_DC_SERVO_1ST,
	FAC_SPECIAL_FUNCTION_DC_SERVO_2ND,
	FAC_SPECIAL_FUNCTION_DC_SERVO_3RD,

	FAC_SPECIAL_FUNCTION_LAST
};

fac_value_t FAC_functions_GET_output(uint8_t functionNumber);
fac_value_t FAC_functions_GET_input(uint8_t functionNumber);
void FAC_functions_SET_output(uint8_t functionNumber, fac_value_t outputValue);
void FAC_functions_update_input(uint8_t functionNumber);	// refresh one slot, used by the functions boilerplate
void FAC_functions_update_inputs(void);	// refresh all the slots, NOT USED, kept for a function that needs more than its own
void FAC_functions_update(uint8_t sFunctionID);
void FAC_functions_init(void);

#endif /* INC_FAC_CODE_MIXES_FUNCTIONS_FAC_FUNCTIONS_H_ */
