/*
 * fac_debug_utils.h
 *
 *  Created on: Aug 1, 2026
 *      Author: Filippo
 */

#ifndef INC_FAC_CODE_FAC_DEBUG_UTILS_H_
#define INC_FAC_CODE_FAC_DEBUG_UTILS_H_

#include "stm32f0xx_hal.h"
#include "config.h"

#ifdef DEBUG_UTILS

#ifdef FUNCTION_CLONOMETER /* CRONOMETER FOR CHECKING EXECUTION TIME */

/* CRONOMETER RESOLUTION */
// TIM6 runs at 48MHz/48 = 1MHz, so 1 tick = 1us, and its counter period is 65535
#define CRONO_TICK_US 1				// duration of a single timer tick, in microseconds
#define CRONO_MAX_MEASURABLE_US 65535	// longest measurable interval, in microseconds

void FAC_debug_utils_crono_init(void);
void FAC_debug_utils_crono_start(void);
uint16_t FAC_debug_utils_crono_stop(void);
#endif

#endif
#endif /* INC_FAC_CODE_FAC_DEBUG_UTILS_H_ */
