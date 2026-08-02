/*
 * LSM6DS3.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Filippo Castellan
 */

#include "stm32f0xx_hal.h"
#include "Libraries/LSM6DS3.h"
#include "iwdg.h"

static const uint8_t DevAddress = 0x6A;	// 0x6A if SA0 = LOW, 0x6B if SA0 = HIGH (shift to left by one bit: 0xD4 - 0xD6)

// The six gyroscope output registers (0x22 to 0x27) are immediately followed by the six
// accelerometer ones (0x28 to 0x2D), so one burst read starting at OUTX_L_G brings home both
// sensors, all three axis each, in a single i2c transaction
#define AXIS_BLOCK_SIZE (LSM6DS3_AXIS_NUMBER * 2)	// two bytes per axis, low byte first
#define BOTH_SENSORS_BLOCK_SIZE (AXIS_BLOCK_SIZE * 2)

// A power of two on purpose: the average is then a shift instead of a call to __aeabi_idiv, which
// is what a division is on a Cortex-M0. 128 samples spaced just above the 2.4ms of the 416Hz
// output data rate cost about 400ms of boot and divide the residual offset error by 11
#define GYRO_OFFSET_SAMPLES 128
#define GYRO_OFFSET_SAMPLE_PERIOD 3	// ms, above the 2.4ms of the output data rate so no sample is read twice

/* STATIC FUNCTION PROTORYPES */
static HAL_StatusTypeDef read_registers(LSM6DS3 *LSM6DS3object, uint8_t regFirstAddress, uint8_t *data, uint16_t size);
static HAL_StatusTypeDef write_register(LSM6DS3 *LSM6DS3object, uint8_t regAddress, uint8_t data);
static int32_t saturate_to_int16(int32_t value);
static int32_t divide_rounded(int32_t numerator, int32_t denominator);
static int16_t decode_axis(const uint8_t *data, uint8_t axis);
static void decode_three_axes(const uint8_t *data, int16_t *destination, const int16_t *offsets);

// FUNCTIONS FOR THE DRIVER

/**
 * @brief	Read one or more consecutive registers into a caller supplied buffer
 * @note	Consecutive registers come out in one transaction thanks to IF_INC, set in CTRL3_C by
 * 			LSM6DS3_init: without it the sensor would return the same register over and over
 * @retval	the status of the transfer, which every caller must check: a read that is not verified
 * 			leaves the previous values in place and looks exactly like a sensor standing still
 */
static HAL_StatusTypeDef read_registers(LSM6DS3 *LSM6DS3object, uint8_t regFirstAddress, uint8_t *data, uint16_t size) {
	return HAL_I2C_Mem_Read(LSM6DS3object->i2c, DevAddress << 1, regFirstAddress, 1, data, size, TIMEOUT_I2C);
}

static HAL_StatusTypeDef write_register(LSM6DS3 *LSM6DS3object, uint8_t regAddress, uint8_t data) {
	return HAL_I2C_Mem_Write(LSM6DS3object->i2c, DevAddress << 1, regAddress, 1, &data, 1, TIMEOUT_I2C);
}

/**
 * @brief	Keep a value inside what an int16_t can hold
 * @retval	value limited to [-32768, +32767]
 */
static int32_t saturate_to_int16(int32_t value) {
	if (value > INT16_MAX)
		return INT16_MAX;
	if (value < INT16_MIN)
		return INT16_MIN;
	return value;
}

/**
 * @brief	Integer division rounded to the nearest instead of truncated toward zero
 * @note	Truncation would bias every offset toward zero by up to one count, and the offset is
 * 			the one number that is subtracted from every reading for the rest of the run
 * @retval	numerator / denominator rounded to the nearest, 0 if the denominator is 0
 */
static int32_t divide_rounded(int32_t numerator, int32_t denominator) {
	if (denominator == 0)
		return 0;
	if (numerator >= 0)
		return (numerator + denominator / 2) / denominator;
	return (numerator - denominator / 2) / denominator;
}

/**
 * @brief	Rebuild one signed axis value out of its two bytes in a burst read buffer
 * @note	The sensor is little endian (BLE = 0 in CTRL3_C), so every axis comes low byte first
 * @param	axis	the position of the axis inside the buffer, not its address
 * @retval	the raw count of that axis
 */
static int16_t decode_axis(const uint8_t *data, uint8_t axis) {
	return (int16_t) (data[axis * 2 + 1] << 8 | data[axis * 2]);
}

/**
 * @brief	Turn a six byte block into the three axis of one sensor, correcting them if asked
 * @note	The correction is summed on int32_t and then saturated. On int16_t a reading close to
 * 			the negative full scale plus a negative offset wraps around into a large positive
 * 			value: at +-2000dps that is the gyroscope reporting a full speed spin the wrong way
 * @param	offsets	the per axis correction to add, or NULL when there is nothing to correct
 */
static void decode_three_axes(const uint8_t *data, int16_t *destination, const int16_t *offsets) {
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++) {
		int32_t value = decode_axis(data, i);
		if (offsets != NULL)
			value += offsets[i];
		destination[i] = (int16_t) saturate_to_int16(value);
	}
}

// FUNCTIONS

/**
 * @brief	Initialize the basic sensor
 * @note	Identifies the chip and enables BDU and IF_INC, the two bits every burst read below
 * 			depends on. It does not touch the i2c peripheral: the bus is shared with the eeprom
 * 			and is brought up once by MX_I2C1_Init, a device driver has no business resetting it
 * @retval	HAL_ERROR if nobody answers on the bus or if the chip is not one of the two known ones
 */
HAL_StatusTypeDef LSM6DS3_init(LSM6DS3 *LSM6DS3object, I2C_HandleTypeDef *hi2c) {
	HAL_Delay(15);	// wait 15ms to be sure that the sensor is fully on
	LSM6DS3object->i2c = hi2c;
	LSM6DS3object->who_am_i = 0;
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++) {
		LSM6DS3object->accel[i] = 0;
		LSM6DS3object->gyro[i] = 0;
		LSM6DS3object->gyro_offsets[i] = 0;
	}

	uint8_t identity = 0;
	if (read_registers(LSM6DS3object, WHO_AM_I, &identity, 1) != HAL_OK) {
		return HAL_ERROR;	// nothing answered at this address
	}
	LSM6DS3object->who_am_i = identity;
	if (identity != WHO_AM_I_VALUE_LSM6DS3 && identity != WHO_AM_I_VALUE_LSM6DS3TRC) {
		return HAL_ERROR;	// something answered, but it is not a device this driver can talk to
	}

	uint8_t settings = 0b01000100;	// BDU (output registers hold still until both their bytes are read), IF_INC (register address auto increment, what makes a burst read possible)
	return write_register(LSM6DS3object, CTRL3_C, settings);
}

HAL_StatusTypeDef LSM6DS3_init_accel(LSM6DS3 *LSM6DS3object) {
	HAL_StatusTypeDef stat;
	// init freq and scale accel
	uint8_t settings = 0b01100100;	// 0110 (416Hz), 01 (+-16g, so LSM6DS3_ACCEL_SENSITIVITY_UG), 00 (widest anti aliasing filter bandwidth)
	stat = write_register(LSM6DS3object, CTRL1_XL, settings);
	HAL_Delay(20);
	return stat;
}

HAL_StatusTypeDef LSM6DS3_init_gyro(LSM6DS3 *LSM6DS3object) {
	HAL_StatusTypeDef stat;
	// init freq and scale accel
	uint8_t settings = 0b01101100;	// 0110 (416Hz), 11 (+-2000dps, so LSM6DS3_GYRO_SENSITIVITY_MDPS), 00 (not applicable)
	stat = write_register(LSM6DS3object, CTRL2_G, settings);
	HAL_Delay(20);
	return stat;
}

/**
 * @brief	Refresh every axis of both sensors with a single i2c transaction
 * @note	This is the cheap way in, and the one normal operation should use: the gyroscope and
 * 			the accelerometer output registers are contiguous, so asking for both costs one
 * 			transaction instead of the six that reading each axis on its own used to take
 * @retval	the status of the transfer. On a failure the values already in the object are left
 * 			untouched, so the caller decides whether stale data is better than no data
 */
HAL_StatusTypeDef LSM6DS3_update_all_values(LSM6DS3 *LSM6DS3object) {
	uint8_t data[BOTH_SENSORS_BLOCK_SIZE];
	HAL_StatusTypeDef stat = read_registers(LSM6DS3object, OUTX_L_G, data, BOTH_SENSORS_BLOCK_SIZE);
	if (stat != HAL_OK) {
		return stat;
	}
	decode_three_axes(data, LSM6DS3object->gyro, LSM6DS3object->gyro_offsets);
	decode_three_axes(&data[AXIS_BLOCK_SIZE], LSM6DS3object->accel, NULL);
	return HAL_OK;
}

HAL_StatusTypeDef LSM6DS3_update_accelerometer_all_values(LSM6DS3 *LSM6DS3object) {
	uint8_t data[AXIS_BLOCK_SIZE];
	HAL_StatusTypeDef stat = read_registers(LSM6DS3object, OUTX_L_XL, data, AXIS_BLOCK_SIZE);
	if (stat != HAL_OK) {
		return stat;
	}
	decode_three_axes(data, LSM6DS3object->accel, NULL);
	return HAL_OK;
}

HAL_StatusTypeDef LSM6DS3_update_gyroscope_all_values(LSM6DS3 *LSM6DS3object) {
	uint8_t data[AXIS_BLOCK_SIZE];
	HAL_StatusTypeDef stat = read_registers(LSM6DS3object, OUTX_L_G, data, AXIS_BLOCK_SIZE);
	if (stat != HAL_OK) {
		return stat;
	}
	decode_three_axes(data, LSM6DS3object->gyro, LSM6DS3object->gyro_offsets);
	return HAL_OK;
}

/**
 * @brief	Calculate new offset for gyro
 * @note	A true average of GYRO_OFFSET_SAMPLES readings, accumulated on int32_t. It used to be
 * 			written as "average = (average + sample) / 2" inside the loop, which is not an average
 * 			at all but an exponential filter with a weight of one half: whatever the sample count,
 * 			its residual noise is that of averaging three readings, and every division truncated
 * 			a little more of the result away
 * @note	The robot must stand still for the whole call, there is no way to tell a zero rate
 * 			offset from a real rotation. Blocks for about 400ms and refreshes the watchdog
 * @retval	HAL_ERROR if any reading failed, and in that case the offsets are left at zero: half
 * 			an average is worse than none, it would be subtracted from every later reading
 */
HAL_StatusTypeDef LSM6DS3_calculate_offset(LSM6DS3 *LSM6DS3object) {
	int32_t sum[LSM6DS3_AXIS_NUMBER] = { 0 };
	uint8_t data[AXIS_BLOCK_SIZE];

	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		LSM6DS3object->gyro_offsets[i] = 0;	// measure the bare sensor, not what a previous run already corrected

	for (int sample = 0; sample < GYRO_OFFSET_SAMPLES; sample++) {
		if (read_registers(LSM6DS3object, OUTX_L_G, data, AXIS_BLOCK_SIZE) != HAL_OK) {
			return HAL_ERROR;
		}
		for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
			sum[i] += decode_axis(data, i);	// GYRO_OFFSET_SAMPLES full scale readings still fit in int32_t with room to spare
		HAL_Delay(GYRO_OFFSET_SAMPLE_PERIOD);
		HAL_IWDG_Refresh(&hiwdg);	// refresh the watchdog	(~400ms)
	}

	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		LSM6DS3object->gyro_offsets[i] = (int16_t) saturate_to_int16(-divide_rounded(sum[i], GYRO_OFFSET_SAMPLES));

	return HAL_OK;
}
