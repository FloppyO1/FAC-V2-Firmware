/*
 * fac_battery.c
 *
 *  Created on: Jul 31, 2025
 *      Author: filippo-castellan
 */

#include "FAC_Code/fac_battery.h"
#include "main.h"

#include "FAC_Code/fac_adc.h"
#include "FAC_Code/fac_settings.h"
#include "iwdg.h"

static Battery battery;

/* STATIC FUNCTION PROTORYPES */
static void FAC_battery_SET_voltage(uint16_t vbat);
static void FAC_battery_SET_cell_voltage(uint16_t vcell);
static void FAC_battery_SET_type(uint8_t type);
static void FAC_battery_calculate_voltage();
static void FAC_battery_calculate_cell_voltage();
static uint16_t FAC_battery_read_voltage(const uint8_t readings);

/* FUNCTION DEFINITION */

static void FAC_battery_SET_voltage(uint16_t vbat) {
	battery.voltage = vbat;
}

static void FAC_battery_SET_cell_voltage(uint16_t vcell) {
	battery.single_cell_voltage = vcell;
}
static void FAC_battery_SET_type(uint8_t type) {
	battery.type = type;
}

void FAC_battery_SET_calibration_offset(int16_t offset) {
	battery.voltage_calibration_offset = offset;
}

/**
 * @bief 	Return the calculated Vbat
 * @retval 	Return the calculate Vbat with the format: 6.253V = 6253mV
 */
uint16_t FAC_battery_GET_voltage() {
	FAC_battery_calculate_voltage();
	return battery.voltage;
}

/**
 * @bief 	Return the calculated voltage of a single cell of the pack
 * @note 	Vcell with the format: 3.812V = 3812mV
 * @note	On USB power or on an unrecognized pack the cell count is unknown, so the pack voltage is returned
 */
uint16_t FAC_battery_GET_cell_voltage() {
	FAC_battery_calculate_cell_voltage();
	return battery.single_cell_voltage;
}

/**
 * @bief 	Return the calculated battery type
 * @retval 	Return the calculate battery type enum value (1,2,3,4S, USB, UNKNOWN)
 */
uint16_t FAC_battery_GET_type(uint16_t vbat) {
	FAC_battery_calculate_type(vbat);
	return battery.type;
}
/**
 * @bief 	Return the calculated battery calibration voltage
 * @retval 	Return the calculate battery voltage calibration offset
 */
int16_t FAC_battery_GET_calibration_offset() {
	return battery.voltage_calibration_offset;
}

/**
 * @bief 	Read the voltage of the battery from the adc reading
 * @note 	Vbat with the format: 6.253V = 6253mV !! without calibration !!
 * @retval	unsigned int mV voltage
 */
static uint16_t FAC_battery_read_voltage(const uint8_t readings) {
	float vbat = 0;
	for (uint8_t i = 0; i < readings; i++) {
		float resolution = (float) FAC_adc_GET_resolution();
		float uVref = (float) FAC_adc_GET_Vref_in_uV();
		float adc = (float) FAC_adc_get_raw_channel_value(ADC_BATTERY_CHANNEL);
		float divider_ratio = (float) battery.voltage_divider_ratio;
		vbat += ((uVref / resolution) * adc * divider_ratio) / 1000000;
	}
	vbat = vbat / (float) readings;

	battery.voltage_uncalibrated = (uint16_t) vbat;

	return ((uint16_t) vbat);
}


/**
 * @bief 	Calculate the voltage of the battery from the adc reading
 * @note 	Vbat with the format: 6.253V = 6253mV
 */
static void FAC_battery_calculate_voltage() {
	int16_t vbat = FAC_battery_read_voltage(5);
	vbat = vbat + FAC_battery_GET_calibration_offset();
	FAC_battery_SET_voltage((uint16_t) vbat);
}

/**
 * @bief 	Calculate the voltage of a single cell of the battery pack
 * @note 	battery.type holds the number of cells detected at the startup (BATTERY_TYPE_1S..4S == 1..4)
 * @note	USB power (BATTERY_TYPE_USB == 0) and an unrecognized pack (BATTERY_TYPE_NONE) have no valid
 * 			cell count, so the divider is forced to 1 (a 0 divider would be a division by zero)
 */
static void FAC_battery_calculate_cell_voltage() {
	uint16_t vbat = FAC_battery_GET_voltage();
	uint8_t cellCount = battery.type;
	if (cellCount < BATTERY_TYPE_1S || cellCount > BATTERY_TYPE_4S)
		cellCount = 1;
	FAC_battery_SET_cell_voltage(vbat / cellCount);
}

/**
 * @bief 	Calculate the battery type
 */
void FAC_battery_calculate_type(uint16_t vbat) {
	if (vbat
			>= NOMINAL_USB_VOLTAGE - TYPIC_DIODE_VOLTAGE_DROP
					- USB_POWER_TOLLERACE
			&& vbat <= NOMINAL_USB_VOLTAGE + USB_POWER_TOLLERACE) {
		FAC_battery_SET_type(BATTERY_TYPE_USB);
		return;
	} else {
		uint8_t temp = vbat / (NOMINAL_BATTERY_LEVEL - TYPIC_BATTERY_RANGE);
		uint16_t bottomRange = temp
				* (NOMINAL_BATTERY_LEVEL - TYPIC_BATTERY_RANGE);
		uint16_t topRange = temp
				* (NOMINAL_BATTERY_LEVEL + TYPIC_BATTERY_RANGE);
		if (vbat >= bottomRange && vbat <= topRange) {
			FAC_battery_SET_type(temp);
		} else {
			FAC_battery_SET_type(BATTERY_TYPE_NONE);
		}
	}
}

/**
 * @brief	Initialize the values of the battery struct
 */
void FAC_battery_init() {
	battery.type = BATTERY_TYPE_USB;
	battery.voltage = 0;
	battery.voltage_uncalibrated = 0;
	battery.single_cell_voltage = 0;
	battery.low_battery_state = FALSE;
	battery.cut_off_state = FALSE;
	battery.voltage_divider_ratio = 7692;
	battery.voltage_calibration_offset = 0;
}
