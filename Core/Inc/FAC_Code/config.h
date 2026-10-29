/*
 * config.h
 *
 *  Created on: Jul 31, 2025
 *      Author: filippo-castellan
 */

#ifndef INC_FAC_CODE_CONFIG_H_
#define INC_FAC_CODE_CONFIG_H_

#ifdef DEBUG
//#define IM_TESTING_FAC_TOOL			// if this is defined the target use only the mcu and eeprom, used for testing settings and nothing else.
//#define SERIAL_DEBUG	// activate the serial debug prints

/* DEBUG UTILS */
// comment and uncomment to activate debug utils functions
#define DEBUG_UTILS
#ifdef DEBUG_UTILS
#define FUNCTION_CLONOMETER

/* CRONOMETER MEASUREMENT POINTS */
// comment and uncomment to choose which file is measured
// !! ONLY ONE AT A TIME !! the cronometer has a single TIM6 counter, so a measure started inside
// another one resets it and both the results are wrong
#define CRONOMETER_ENTIRE_FAC_APP // the entire main loop of the app
//#define CRONOMETER_FAC_APP		// blocks of FAC_app_main_loop (the states of the app)
//#define CRONOMETER_FAC_MAPPER		// blocks of FAC_mapper_apply_to_devices (mix update and devices)

// the sum counts how many are enabled: with && the check would only fire when ALL of them are
// defined, while the hazard is already there with any TWO of them (defined() is 1 or 0 in a #if)
#if (defined(CRONOMETER_ENTIRE_FAC_APP) + defined(CRONOMETER_FAC_APP) + defined(CRONOMETER_FAC_MAPPER)) > 1
#error "Cronometer measures cannot be nested: enable only one CRONOMETER_* at a time"
#endif
#endif
#endif

/* FIRMWARE VERSION */
// VERSION NOTE (in addition to the previews one):
// Now the FAC will always boot up and is forced to DISARMED state until a valid signal came from the receiver
// than the arming channel/setting will decide the state
// VERSION NOTE 2.0.8:
// The mix/function/mapper chain no longer uses floats (fac_math.h, integer only, no fpu needed)
// EEPROM writes now refresh the watchdog, a large save no longer resets the board halfway through
// Applying settings from the fac tool no longer clears the live receiver channels or glitches the
// motor/servo outputs, it only reconfigures what actually changed
// Unmapped motors are now forced to 0 every loop, same as unmapped servos already were
// VERSION NOTE 2.1.0:
// The imu is no longer calibrated at boot but when the robot is armed, because at power up it may
// still be in the hand of its operator and offsets measured on a moving body are wrong for the
// whole run. The calibration also measures which way is really up and builds a world frame out of
// it, so a mix gets a truly vertical z whatever the mounting angle of the board (FAC_IMU_calibrate,
// FAC_IMU_GET_*_level_raw, FAC_IMU_GET_yaw_rate_mdps, FAC_IMU_GET_heading)
// A board whose imu failed to init still arms immediately, the imu stays non fatal by design
// NOTE ON THIS BUMP: no setting was added, removed or renumbered, so the FAC Tool is unaffected.
// The version bump does change FIRMWARE_VERSION_TAG though, and FAC_settings_init rewrites the
// eeprom with the defaults whenever that tag moves: the user configuration is reset to defaults on
// the first boot after this update, which the 10 fast boot blinks announce
// VERSION NOTE 2.2.0:
// A mix has 6 inputs instead of 8. The codes of inputs 7 and 8 (channel and reversal) now hold 4
// numeric parameters of the mix (FAC_SETTINGS_CODE_MIX_PARAM1..4, int16_t), read by a mix with
// FAC_mixes_GET_param(). Settings renumbered: the reversals of inputs 1..6 moved to codes 27..32,
// the parameters are 33..36; the table still has 64 codes and every other code is unchanged.
// The FAC Tool picks the right table from the version. Settings reset to defaults on the first boot
#define FIRMWARE_VERSION_MAJOR 2	// [MAX 9] 	add one here, if the changes are substantial and not backwards compatible
#define FIRMWARE_VERSION_MINOR 2	// [MAX 99] add one here, if new backwards compatible features have been added
#define FIRMWARE_VERSION_PATCH 1	// [MAX 99] add one here, if any issues have been fixed without adding any features

#define FIRMWARE_VERSION_TAG  (uint8_t)((FIRMWARE_VERSION_MAJOR * 101U + FIRMWARE_VERSION_MINOR * 7U + FIRMWARE_VERSION_PATCH * 3U) % 255U)

// The version as it is stored in FAC_SETTINGS_CODE_FIRMWARE_VERSION and sent to the fac tool:
// 20513 -> 2.05.13 <major>,<minor>,<patch>. It is the default of that setting too, so a factory
// reset writes the version of the firmware that is actually running and not an old hardcoded one
#define FIRMWARE_VERSION_VALUE (uint16_t)((FIRMWARE_VERSION_MAJOR * 10000U) + (FIRMWARE_VERSION_MINOR * 100U) + FIRMWARE_VERSION_PATCH)

/* RECEIVER */
#define RECEIVER_CHANNEL_RESOLUTION 1000	// max 2000, because the timer tick is 0.5us (500ns)
#define RECEIVER_CHANNELS_NUMBER 8

/* MOTORS */
#define MOTOR_SPEED_RESOLUTION RECEIVER_CHANNEL_RESOLUTION		// use always max resolution of the receiver

/* SERVOS */
#define SERVO_POSITION_RESOLUTION RECEIVER_CHANNEL_RESOLUTION	// use always max resolution
#define SERVO_FREQUENCY 50	// it can be selected from the settings tool (?)

/* FAC APP */
#define ARMING_THRESHOLD 80				// 80% of the max receiver number

#define DISARMED_LED_PERIOD	1000			// in milliseconds
#define DISARMED_TONE_PERIOD 500
#define DISARMED_TONE_DURATION 100

#define LOW_BATTERY_LED_PERIOD 500

#define CUT_OFF_LED_PERIOD	200
#define CUT_OFF_TONE_PERIOD 300
#define CUT_OFFTONE_DURATION 100

//#define NORMAL_LED_PERIOD	3000		// in milliseconds

/* IMU CALIBRATION WHILE ARMING */
// The gyro offsets and the orientation reference are measured while leaving DISARMED, when the
// robot is presumed placed in the arena and standing still. One attempt blocks for about 400ms
#define FAC_ARMING_CALIBRATION_RETRY_PERIOD 200		// in milliseconds, idle time between two attempts so the loop can breathe
#define FAC_ARMING_CALIBRATION_ATTEMPTS 5			// with an arming channel: arm anyway after this many failures
#define FAC_ARMING_CALIBRATION_STILLNESS_TIMEOUT 10000	// in milliseconds, with NO arming channel: arm anyway after this long
#define FAC_ARMING_CALIBRATION_OK_TONE_FREQ 1568	// one short high beep, the calibration passed
#define FAC_ARMING_CALIBRATION_OK_TONE_DURATION 80
#define FAC_ARMING_CALIBRATION_FAIL_TONE_FREQ 392	// two long low beeps, armed with the previous offsets
#define FAC_ARMING_CALIBRATION_FAIL_TONE_DURATION 150
#define FAC_ARMING_CALIBRATION_FAIL_LED_PERIOD 100
#define FAC_ARMING_CALIBRATION_FAIL_BLINKS 6

/* IMU AXIS CONVENTION */
// A mems accelerometer measures specific force, so at rest the axis pointing upwards is the one
// reading +1g, and the whole level frame is built on that. Leave this at +1 unless a board turns
// out to be populated with a part answering with the opposite sign: lay the FAC flat with the
// components facing up and check that FAC_IMU_GET_accel_Z_mg() reads about +1000. The orientation
// viewer in the TOOLS folder of the parent repo shows it directly, and there is no way to detect
// it from the data alone: a board at rest reading (0, 0, +1g) is indistinguishable from an upside
// down one under the opposite convention.
// It is applied to the ACCELEROMETER ALONE and as close to the driver as possible, so that
// everything downstream always sees the standard convention. Do NOT "fix" the sign inside
// FAC_IMU_BUILD_frame instead: negating up there on its own leaves the accelerometer path with a
// single negation instead of the two that used to cancel, and is_upside_down comes out true while
// the robot is the right way up. The long comment in fac_imu.c walks through why
#define IMU_ACCEL_SIGN -1	// +1 standard convention, -1 inverts every accelerometer axis

/* IMU CALIBRATION THRESHOLDS */
// !! STARTING POINTS, MEASURED NOISE WISE AND TO BE VERIFIED ON HARDWARE !!
// They decide whether a still measure is trustworthy. Too tight and a robot on a vibrating arena
// floor never calibrates, too loose and a robot still being carried calibrates on a wrong vertical
#define IMU_CALIBRATION_MAX_GYRO_SPAN 50	// raw counts, max - min over the window, 50 is about 3.5dps
#define IMU_CALIBRATION_MAX_ACCEL_SPAN 100	// raw counts, max - min over the window, 100 is about 49mg
#define IMU_CALIBRATION_ACCEL_MIN 1639		// raw counts, 0.8g: below it the reading is not gravity alone
#define IMU_CALIBRATION_ACCEL_MAX 2459		// raw counts, 1.2g: above it the robot is being accelerated
#define IMU_CALIBRATION_ACCEL_LIMIT 8192	// raw counts, about 4g. NOT a taste threshold: it is the overflow
											// guard of the sum of the squares, checked BEFORE squaring
#define IMU_CALIBRATION_VERTICAL_LIMIT 900	// on the FAC_IMU_UNIT_SCALE scale, about 26 degrees: past it the
											// board own +x is too close to vertical to define a forward

/* IMU HEADING */
#define FAC_IMU_HEADING_MAX_GAP 100			// in milliseconds, a longer hole is resynchronised and not integrated

#endif /* INC_FAC_CODE_CONFIG_H_ */
