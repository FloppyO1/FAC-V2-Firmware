/*
 * fac_gyro.c
 *
 *  Created on: Sep 17, 2025
 *      Author: Filippo Castellan
 */

#include "FAC_Code/fac_imu.h"
#include "FAC_Code/mixes_functions/fac_math.h"	// the integer primitives the unit conversions are built from
#include "main.h"
#include "i2c.h"
#include "iwdg.h"

// A sensor that dropped off the bus must not cost a blocking i2c timeout on every loop pass, so
// after a failed read it is left alone for this long before being tried again
#define IMU_RETRY_PERIOD 1000	// ms

static Gyro gyro;

/* STATIC FUNCTION PROTORYPES */
static void FAC_IMU_SET_status(HAL_StatusTypeDef gyro_status);
static LSM6DS3* FAC_IMU_GET_LSM6DS3_object(void);

/* STATIC FUNCTION */
static void FAC_IMU_SET_status(HAL_StatusTypeDef gyro_status) {
	gyro.gyro_status = gyro_status;
}

static LSM6DS3* FAC_IMU_GET_LSM6DS3_object(void) {
	return &gyro.LSM6DS3object;
}

/* FUNCTION DEFINITION */

HAL_StatusTypeDef FAC_IMU_GET_status(void) {
	return gyro.gyro_status;
}

/**
 * @brief	Refresh every axis of both sensors, and it is the only call that touches the bus
 * @note	Call it once before reading the accessors below: they hand back what this left in the
 * 			object and cost nothing, so any number of consumers in the same loop pass share a
 * 			single i2c transaction. Reading each axis on its own used to cost one transaction each
 * @note	Three guards keep it cheap to call. Nothing happens before the init chain completed,
 * 			so a missing sensor is never talked to. A second call inside the same millisecond
 * 			returns at once, which costs nothing in freshness: at the 416Hz output data rate a new
 * 			sample only lands every 2.4ms anyway. After a failure the sensor is retried once every
 * 			IMU_RETRY_PERIOD instead of on every pass
 * @note	A failed read leaves the previous values in place and marks the status, so a consumer
 * 			that checks FAC_IMU_GET_status can tell fresh data from stale
 */
void FAC_IMU_update(void) {
	if (!gyro.is_initialized) {
		return;	// the sensor is missing or was never configured, there is nothing to read
	}

	uint32_t now = HAL_GetTick();
	if (now == gyro.last_update_tick) {
		return;	// already read in this millisecond, the sensor has no new sample to give yet
	}
	if (FAC_IMU_GET_status() == HAL_ERROR && (now - gyro.last_update_tick) < IMU_RETRY_PERIOD) {
		return;	// still inside the backoff, do not pay a blocking timeout for a sensor known to be down
	}

	gyro.last_update_tick = now;
	FAC_IMU_SET_status(LSM6DS3_update_all_values(FAC_IMU_GET_LSM6DS3_object()));
}

int16_t FAC_IMU_GET_accel_raw(uint8_t axis) {
	if (axis >= LSM6DS3_AXIS_NUMBER)
		return 0;
	return FAC_IMU_GET_LSM6DS3_object()->accel[axis];
}

int16_t FAC_IMU_GET_gyro_raw(uint8_t axis) {
	if (axis >= LSM6DS3_AXIS_NUMBER)
		return 0;
	return FAC_IMU_GET_LSM6DS3_object()->gyro[axis];
}

/**
 * @brief	Acceleration of one axis in milli g, so 1g = 1000
 * @note	One division each, and only for the callers that want engineering units: a mix is
 * 			usually better off normalizing the raw count with FAC_math_from_range instead
 * @note	The widest product is 32767 * 488, far inside int32_t
 * @retval	the axis in mg, +-16000 at the programmed full scale
 */
int32_t FAC_IMU_GET_accel_X_mg(void) {
	return FAC_math_mul_scaled(FAC_IMU_GET_accel_raw(X_AXIS), LSM6DS3_ACCEL_SENSITIVITY_UG, 1000);
}

int32_t FAC_IMU_GET_accel_Y_mg(void) {
	return FAC_math_mul_scaled(FAC_IMU_GET_accel_raw(Y_AXIS), LSM6DS3_ACCEL_SENSITIVITY_UG, 1000);
}

int32_t FAC_IMU_GET_accel_Z_mg(void) {
	return FAC_math_mul_scaled(FAC_IMU_GET_accel_raw(Z_AXIS), LSM6DS3_ACCEL_SENSITIVITY_UG, 1000);
}

/**
 * @brief	Angular rate of one axis in milli degrees per second, so 1dps = 1000
 * @note	Milli and not whole degrees per second on purpose: the sensitivity is a whole number of
 * 			mdps per count, so the conversion is a multiplication and costs no division at all,
 * 			and no resolution is thrown away on the slow rates
 * @note	The widest product is 32767 * 70, far inside int32_t
 * @retval	the axis in mdps, +-2000000 at the programmed full scale
 */
int32_t FAC_IMU_GET_gyro_X_mdps(void) {
	return (int32_t) FAC_IMU_GET_gyro_raw(X_AXIS) * LSM6DS3_GYRO_SENSITIVITY_MDPS;
}

int32_t FAC_IMU_GET_gyro_Y_mdps(void) {
	return (int32_t) FAC_IMU_GET_gyro_raw(Y_AXIS) * LSM6DS3_GYRO_SENSITIVITY_MDPS;
}

int32_t FAC_IMU_GET_gyro_Z_mdps(void) {
	return (int32_t) FAC_IMU_GET_gyro_raw(Z_AXIS) * LSM6DS3_GYRO_SENSITIVITY_MDPS;
}

/*
 * @brief	Init the accelerometer of the chip
 * @note	Does nothing when the sensor is already known to be missing or wrong. Overwriting the
 * 			status here is what used to disarm the identity check of FAC_IMU_init: a chip that
 * 			failed WHO_AM_I but still acknowledged this write came back as HAL_OK
 */
void FAC_IMU_init_accelerometer(void) {
	if (FAC_IMU_GET_status() == HAL_ERROR) {
		return;
	}
	FAC_IMU_SET_status(LSM6DS3_init_accel(FAC_IMU_GET_LSM6DS3_object()));
	HAL_Delay(100);
}

/*
 * @brief	Init the gyroscope of the chip
 * @note	Last step of the init chain, so this is where the sensor is declared usable and
 * 			FAC_IMU_update is allowed to talk to it
 */
void FAC_IMU_init_gyroscope(void) {
	if (FAC_IMU_GET_status() == HAL_ERROR) {
		return;
	}
	FAC_IMU_SET_status(LSM6DS3_init_gyro(FAC_IMU_GET_LSM6DS3_object()));
	HAL_Delay(100);
	if (FAC_IMU_GET_status() != HAL_ERROR) {
		gyro.is_initialized = TRUE;
	}
}

/*
 * @brief	Compute new gyro offsets
 * @note	Blocks for about 400ms with the led held on, the robot must stand still meanwhile
 */
void FAC_IMU_compute_gyro_offset(void) {
	if (FAC_IMU_GET_status() == HAL_ERROR) {
		return;
	}
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	FAC_IMU_SET_status(LSM6DS3_calculate_offset(FAC_IMU_GET_LSM6DS3_object()));
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/*
 * @brief		Initialize the gyro/accel LSM6DS3
 * @IMPORTANT	If status is HAL_ERROR the mix or special function must manage this problem
 */
HAL_StatusTypeDef FAC_IMU_init(void) {
	HAL_StatusTypeDef gyro_status;
	HAL_Delay(200);	// wait for the imu startup
	gyro.is_initialized = FALSE;
	gyro.last_update_tick = 0;
	gyro_status = LSM6DS3_init(FAC_IMU_GET_LSM6DS3_object(), &hi2c1);
	FAC_IMU_SET_status(gyro_status);	// on success too: it used to be set only on failure, and the good case worked only because the struct is zero initialized and HAL_OK is 0
	return gyro_status;
}
