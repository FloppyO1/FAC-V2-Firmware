/*
 * fac_debug_utils.c
 *
 *  Created on: Aug 1, 2026
 *      Author: Filippo
 */
#include "FAC_Code/fac_debug_utils.h"
#include "tim.h"

#ifdef DEBUG_UTILS

#ifdef FUNCTION_CLONOMETER /* CRONOMETER FOR CHECKING EXECUTION TIME */

/* STATIC LOCAL FUNCTIONS */
static void FAC_debug_utils_crono_reset(void);
/* STATIC LOCAL VARIABLES */
static uint16_t t_crono_start = 0;	// start timestamp of cronometer
static uint16_t t_crono_stop = 0;	// stop timestamp of cronometer

/* FUNCTIONS DEFINITIONS */

/**
 * @brief	Initialize the cronometer
 * @note	TIM6 is configured by CubeMX (MX_TIM6_Init) as a free running 16 bit counter with
 * 			a 1MHz clock, so it is only stopped and cleared here
 */
void FAC_debug_utils_crono_init(void) {
	HAL_TIM_Base_Stop(&htim6);		// the cronometer counts only between a start and a stop
	FAC_debug_utils_crono_reset();	// reset the timer and timestamps stored
}

/**
 * @brief	Start a new time measurement
 * @note	A measurement still running is discarded, the last start always wins
 * @IMPORTANT	The measurement is not reentrant: there is a single counter, so a crono_start
 * 				inside an already timed block breaks the outer measurement
 */
void FAC_debug_utils_crono_start(void) {
	HAL_TIM_Base_Stop(&htim6);		// stop a measurement eventually still running
	FAC_debug_utils_crono_reset();	// reset the timer and timestamps stored
	HAL_TIM_Base_Start(&htim6);		// start the timer
	t_crono_start = __HAL_TIM_GET_COUNTER(&htim6);
}

/**
 * @brief	Stop the measurement started by FAC_debug_utils_crono_start and return its duration
 * @note	The duration is expressed in timer ticks, 1 tick = CRONO_TICK_US (1us)
 * @note	The update flag of TIM6 is raised by the counter overflow and is cleared on every
 * 			start, so here it means that the measured block lasted more than one full counter period
 * @retval	elapsed time in microseconds, 0 if overflow, means the value is bigger
 * 			than CRONO_MAX_MEASURABLE_US (65'535 us)
 */
uint16_t FAC_debug_utils_crono_stop(void) {
	t_crono_stop = __HAL_TIM_GET_COUNTER(&htim6);
	HAL_TIM_Base_Stop(&htim6);
	// calculate the elapsed time
	uint16_t duration = 0;
	if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET) {
		duration = 0;
	} else {
		duration = t_crono_stop - t_crono_start;
	}
	return duration;
}

/**
 * @brief	Bring the cronometer back to its idle state
 * @note	The update flag has to be cleared too, otherwise a single overflow would make
 * 			every following measurement return 0
 */
static void FAC_debug_utils_crono_reset(void) {
	__HAL_TIM_SET_COUNTER(&htim6, 0); // reset timer value to 0
	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);	// clear the overflow flag of the previous measurement
	t_crono_start = 0;
	t_crono_stop = 0;
}
#endif

#endif

