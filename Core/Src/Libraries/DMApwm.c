/*
 * DMApwm.c
 *
 *  Created on: Dec 23, 2024
 *      Author: Filippo Castellan
 *
 *  Some of the code is taken from https://github.com/javiBajoCero/softPWMtutorial.git
 *  https://www.hackster.io/javier-munoz-saez/all-pins-as-pwm-at-the-same-time-baremetal-stm32-1df86f#toc-dma-3
 */
#include "Libraries/DMApwm.h"

// ---------------------------------------------------------------------------------------------------------------
// 				!!!!!!!!!!!!!	 ATTENCTION THIS CODE WORKS ONLY FOR PORTs A AND B	  !!!!!!!!!!!!!!!!!
// ---------------------------------------------------------------------------------------------------------------

/*	HOW TO SET UP TIMERs AND DMAs
 * 1) find 2 timers that can access via DMA to the BSRR register (TIM1, TIM2, TIM4 should be good)
 * 2) set them with a clock source (internal one is ok)
 * 3) on the parameter settings of the each timer under "trigger output" TRGO set to Update Event
 * 4) activate for each timer the DMA
 * 5) set DMA channel to transfer data from memory to peripheral, a Circular array of Words
 * 6) in this file change the timers and DMAs hendle type as described below
 * 7) on the DMApwm.h file set the desired PWM_STEPS and PWM_FREQ as long as TIMER_FREQ
 * 8) you are good to go, all the GPIO where you want the PWM must be part of A or B port and must be setted as GPIO_Output
 */

// SOME SETTINGS ARE INSIDE THE DMApwm.h file
// timer handle type and hdma handle type
// INSERT YOUR HADLE TYPE NAME making find&replace in all the document (Ctrl+F)
extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim1_up;
//extern DMA_HandleTypeDef hdma_tim2_up;

// buffer for the GPIO values (data transfered by DMA)
static uint32_t dataA[PWM_STEPS];
//static uint32_t dataB[PWM_STEPS]; // not used this time

/* LAST DUTY WRITTEN ON EVERY PIN OF THE PORT */
// the buffer keeps the duty of every pin, so rewriting all of it on each call means writing again
// the same values: the last duty is cached to rewrite only what really changes (see setSoftPWM)
// !! PORT A ONLY, like the rest of this file: enabling port B needs its own cache and valid mask !!
static uint16_t lastDutyA[GPIO_PINS_NUMBER];
static uint16_t lastDutyValidA = 0;	// one bit per pin, 0 means the cached duty cannot be trusted

/*
 * @brief	Convert a single pin mask (GPIO_PIN_x) into its bit position
 * @note	Cortex-M0 has no CLZ instruction, so the position is searched with a loop: at most 16
 * 			iterations, nothing compared to the buffer writes this lookup allows to avoid
 * @retval	the pin position [0, GPIO_PINS_NUMBER-1], GPIO_PINS_NUMBER if the mask holds no pin
 */
static uint8_t pinToIndex(uint16_t pin) {
	uint8_t index = 0;
	while (index < GPIO_PINS_NUMBER && !(pin & (uint16_t) (1u << index)))
		index++;
	return index;
}

/*
 * @brief	Apply a duty to a pin writing on the buffer only the entries that really change
 * @note	Every entry of the buffer is a BSRR word: the low half sets the pin, the high half resets
 * 			it. The entries below the duty keep the pin high, the others keep it low, so changing the
 * 			duty only moves that boundary: the entries between the old and the new duty are the only
 * 			ones that need to be touched, and an unchanged duty costs nothing at all
 * @note	The first write of a pin, and the first one after a zeroSoftPWM, has no usable old duty,
 * 			so the whole buffer is written and the pin is marked as cached
 * @IMPORTANT	The DMA reads this buffer while it is written. That was already the case before, and
 * 				it stays safe because each entry is updated with a single word store
 */
static void setSoftPWM(uint16_t pin, uint32_t duty, uint32_t *softpwmbuffer) {	// no external use
	uint8_t pinIndex = pinToIndex(pin);
	if (pinIndex >= GPIO_PINS_NUMBER)
		return;	// not a valid single pin mask, there is nothing to write

	if (duty > PWM_STEPS)
		duty = PWM_STEPS;	// the loops below index the buffer with the duty, out of range would overflow it

	const uint32_t setMask = (uint32_t) pin;			// BSRR low half, the pin goes high
	const uint32_t resetMask = (uint32_t) pin << 16;	// BSRR high half, the pin goes low

	if (!(lastDutyValidA & (uint16_t) (1u << pinIndex))) {	// no usable old duty, write the whole buffer
		for (uint32_t i = 0; i < PWM_STEPS; ++i) {
			if (i < duty)	// set pin
				softpwmbuffer[i] = (softpwmbuffer[i] & ~resetMask) | setMask;
			else	// reset pin
				softpwmbuffer[i] = (softpwmbuffer[i] & ~setMask) | resetMask;
		}
		lastDutyValidA |= (uint16_t) (1u << pinIndex);
	} else {
		uint32_t oldDuty = lastDutyA[pinIndex];
		if (duty > oldDuty) {	// the pin stays high longer, set the entries in between
			for (uint32_t i = oldDuty; i < duty; ++i)
				softpwmbuffer[i] = (softpwmbuffer[i] & ~resetMask) | setMask;
		} else {	// the pin stays high less, reset the entries in between (equal duty = no iteration)
			for (uint32_t i = duty; i < oldDuty; ++i)
				softpwmbuffer[i] = (softpwmbuffer[i] & ~setMask) | resetMask;
		}
	}
	lastDutyA[pinIndex] = (uint16_t) duty;
}

/*
 * @brief	Clear the whole buffer, no pin is driven anymore
 * @IMPORTANT	It throws away what the duty cache describes, so the cache must be invalidated with it
 */
static void zeroSoftPWM(uint32_t softpwmbuffer[]) {
	for (uint32_t i = 0; i < PWM_STEPS; ++i) {
		softpwmbuffer[i] = 0;
	}
	lastDutyValidA = 0;	// the buffer no longer holds the cached duties, force a full rewrite
}

/*
 * @brief	Change the frequency of the DMA pwm generator
 * @note 	can be useful in case of tone change or simply motor drive frequency change
 */
void FAC_DMA_pwm_change_freq(uint16_t freq) {
	htim1.Init.Period = TIMER_FREQ - 1;
	htim1.Instance->ARR = (TIMER_FREQ / (PWM_STEPS * freq)) - 1;
	HAL_TIM_Base_Start(&htim1);
}

void initDMApwm(uint16_t freq) {
	// set the frequency
	htim1.Init.Period = TIMER_FREQ - 1;
	htim1.Instance->ARR = (TIMER_FREQ / (PWM_STEPS * freq)) - 1;
//	htim2.Init.Period = TIMER_FREQ - 1;
//	htim2.Instance->ARR = (TIMER_FREQ / (PWM_STEPS * PWM_FREQ)) - 1;
	// start timers
	HAL_TIM_Base_Start(&htim1);
//	HAL_TIM_Base_Start(&htim2);

	// configure DMAs
	HAL_DMA_Start(&hdma_tim1_up, (uint32_t) &(dataA[0]), (uint32_t) &(GPIOA->BSRR), sizeof(dataA) / sizeof(dataA[0]));
//	HAL_DMA_Start(&hdma_tim2_up, (uint32_t) &(dataB[0]), (uint32_t) &(GPIOB->BSRR), sizeof(dataB) / sizeof(dataB[0]));

	// starts DMAs
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
//	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);

	// set the PWMs to 0
	zeroSoftPWM(dataA);
//	zeroSoftPWM(dataB);
}

uint8_t setDMApwmDuty(GPIO_TypeDef *port, uint16_t pin, uint16_t duty) {
	uint8_t r = 0;
	if (port == GPIOA) {
		setSoftPWM(pin, duty, (uint32_t*) &dataA);
		r = 1;
	}
//	if (port == GPIOB) {
//		setSoftPWM(pin, duty, (uint32_t*) &dataB);
//		r = 1;
//	}
	return r;	// 1 = ok, 0 = no compatible port found
}

