/*
 * fac_gyro.h
 *
 *  Created on: Sep 17, 2025
 *      Author: Filippo Castellan
 */

#ifndef INC_FAC_CODE_FAC_IMU_H_
#define INC_FAC_CODE_FAC_IMU_H_

#include "stm32f0xx_hal.h"
#include "Libraries/LSM6DS3.h"

// The scale of the calibrated frame: 1000 means 1.0, the same "one" the whole firmware works on
// (see fac_math.h). The three basis vectors are unit vectors on this scale, so a component is a
// number in [-1000, +1000] and every product of two of them stays around 10^6, far inside int32_t
#define FAC_IMU_UNIT_SCALE 1000

// One levelled vector, projected on the calibrated frame and kept until the next sensor read.
// It is a cache and not a value computed by FAC_IMU_update on purpose: a robot whose mix never
// asks for levelled data must not pay the nine multiplications and three divisions of a projection
typedef struct FAC_IMU_level_cache {
	int32_t value[LSM6DS3_AXIS_NUMBER];	// x forward, y left, z up
	uint32_t last_update_tick;			// the sensor read this was projected from
	uint8_t is_valid;					// FALSE while nothing was ever cached, or after the frame changed
} FAC_IMU_level_cache;

typedef struct Gyro {
	LSM6DS3 LSM6DS3object;
	HAL_StatusTypeDef gyro_status;	// HAL_OK, or HAL_ERROR	(in case of error the mix or function using it must manage the problem)
	uint32_t last_update_tick;		// when the last read landed, so one transaction serves every consumer of the same loop pass
	uint8_t is_initialized;			// TRUE only once the whole init chain went through, see FAC_IMU_update
	uint8_t is_calibrated;			// TRUE once a still measure passed validation, see FAC_IMU_calibrate

	/* THE CALIBRATED FRAME */
	// The three world axis expressed in BODY coordinates, scale FAC_IMU_UNIT_SCALE. Right handed,
	// z really vertical whatever the mounting angle of the board. Identity until a calibration
	// passes, so the levelled accessors degrade to the board own axis instead of to garbage
	int16_t forward[LSM6DS3_AXIS_NUMBER];	// world +x
	int16_t left[LSM6DS3_AXIS_NUMBER];		// world +y
	int16_t up[LSM6DS3_AXIS_NUMBER];		// world +z, the measured vertical
	FAC_IMU_level_cache accel_level;
	FAC_IMU_level_cache gyro_level;

	/* THE HEADING INTEGRATOR */
	// Integrated by FAC_IMU_GET_heading itself and by nobody else, so a firmware where no mix asks
	// for a heading pays exactly nothing for it
	int32_t heading_udeg;			// micro degrees, kept inside [0, 360000000)
	uint32_t heading_tick;			// when the accumulator was last advanced or resynchronised
	uint8_t is_heading_synced;		// FALSE before the first call, which must resynchronise and not integrate
} Gyro;

HAL_StatusTypeDef FAC_IMU_GET_status(void);
void FAC_IMU_update(void);
int16_t FAC_IMU_GET_accel_raw(uint8_t axis);
int16_t FAC_IMU_GET_gyro_raw(uint8_t axis);
int32_t FAC_IMU_GET_accel_X_mg(void);
int32_t FAC_IMU_GET_accel_Y_mg(void);
int32_t FAC_IMU_GET_accel_Z_mg(void);
int32_t FAC_IMU_GET_gyro_X_mdps(void);
int32_t FAC_IMU_GET_gyro_Y_mdps(void);
int32_t FAC_IMU_GET_gyro_Z_mdps(void);
HAL_StatusTypeDef FAC_IMU_init(void);
void FAC_IMU_init_accelerometer(void);
void FAC_IMU_init_gyroscope(void);

/* CALIBRATION */
uint8_t FAC_IMU_calibrate(void);
uint8_t FAC_IMU_GET_is_calibrated(void);

/* LEVELLED DATA, WORLD FRAME: X FORWARD, Y LEFT, Z UP */
int32_t FAC_IMU_GET_accel_level_raw(uint8_t axis);
int32_t FAC_IMU_GET_gyro_level_raw(uint8_t axis);

/* ATTITUDE */
int16_t FAC_IMU_GET_up(uint8_t axis);
int32_t FAC_IMU_GET_tilt_angle(void);
uint8_t FAC_IMU_GET_is_upside_down(void);
int32_t FAC_IMU_GET_yaw_rate_mdps(void);

/* HEADING (OPT IN, IT IS INTEGRATED ONLY BY THE CALLS BELOW) */
int32_t FAC_IMU_GET_heading(void);
void FAC_IMU_RESET_heading(void);

#endif /* INC_FAC_CODE_FAC_IMU_H_ */
