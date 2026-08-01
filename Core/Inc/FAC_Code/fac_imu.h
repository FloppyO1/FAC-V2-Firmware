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

typedef struct Gyro {
	LSM6DS3 LSM6DS3object;
	HAL_StatusTypeDef gyro_status;	// HAL_OK, or HAL_ERROR	(in case of error the mix or function using it must manage the problem)
} Gyro;

HAL_StatusTypeDef FAC_IMU_GET_status(void);
float FAC_IMU_GET_accel_X(void);
float FAC_IMU_GET_accel_Y(void);
float FAC_IMU_GET_accel_Z(void);
float FAC_IMU_GET_gyro_X(void);
float FAC_IMU_GET_gyro_Y(void);
float FAC_IMU_GET_gyro_Z(void);
HAL_StatusTypeDef FAC_IMU_init(void);
void FAC_IMU_init_accelerometer(void);
void FAC_IMU_init_gyroscope(void);
void FAC_IMU_compute_gyro_offset(void);

#endif /* INC_FAC_CODE_FAC_IMU_H_ */
