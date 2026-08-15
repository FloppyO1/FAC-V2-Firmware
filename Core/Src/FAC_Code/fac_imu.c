/*
 * fac_gyro.c
 *
 *  Created on: Sep 17, 2025
 *      Author: Filippo Castellan
 */

#include "FAC_Code/fac_imu.h"
#include "FAC_Code/config.h"	// the calibration thresholds and the heading gap limit
#include "FAC_Code/mixes_functions/fac_math.h"	// the integer primitives the unit conversions are built from
#include "main.h"
#include "i2c.h"
#include "iwdg.h"

// A sensor that dropped off the bus must not cost a blocking i2c timeout on every loop pass, so
// after a failed read it is left alone for this long before being tried again
#define IMU_RETRY_PERIOD 1000	// ms

// A whole turn in the unit the heading accumulator works in. Micro degrees, because a rate in
// mdps multiplied by a delta in ms lands exactly there with no scaling and no division
#define HEADING_FULL_TURN_UDEG 360000000

// The accelerometer components are clamped to this before being squared for the tilt angle: two
// of them at full scale would already overflow int32_t, and 30000 counts is about 14.6g, a
// regime in which the direction of gravity is not what the reading is made of anyway
#define TILT_COMPONENT_LIMIT 30000

static Gyro gyro;

/* STATIC FUNCTION PROTORYPES */
static void FAC_IMU_SET_status(HAL_StatusTypeDef gyro_status);
static LSM6DS3* FAC_IMU_GET_LSM6DS3_object(void);
static void FAC_IMU_SET_identity_frame(void);
static int32_t FAC_IMU_project(const int16_t world_axis[LSM6DS3_AXIS_NUMBER], const int32_t body[LSM6DS3_AXIS_NUMBER]);
static int32_t FAC_IMU_norm(const int32_t vector[LSM6DS3_AXIS_NUMBER]);
static uint8_t FAC_IMU_normalize(int32_t vector[LSM6DS3_AXIS_NUMBER]);
static uint8_t FAC_IMU_VALIDATE_still_measure(const LSM6DS3_still_measure *measure);
static uint8_t FAC_IMU_BUILD_frame(const int16_t accel_mean[LSM6DS3_AXIS_NUMBER]);
static const FAC_IMU_level_cache* FAC_IMU_GET_accel_level(void);
static const FAC_IMU_level_cache* FAC_IMU_GET_gyro_level(void);
static int32_t FAC_IMU_GET_heading_from_accumulator(void);

/* STATIC FUNCTION */
static void FAC_IMU_SET_status(HAL_StatusTypeDef gyro_status) {
	gyro.gyro_status = gyro_status;
}

static LSM6DS3* FAC_IMU_GET_LSM6DS3_object(void) {
	return &gyro.LSM6DS3object;
}

/*
 * @brief	Set the frame to the board own axis and drop whatever was projected on the old one
 * @note	This is the state before any calibration, and it is what makes the levelled accessors
 * 			degrade gracefully: with the identity frame they return exactly the body values, so a
 * 			mix reading them on an uncalibrated board sees the raw axis instead of zeros or garbage
 */
static void FAC_IMU_SET_identity_frame(void) {
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++) {
		gyro.forward[i] = 0;
		gyro.left[i] = 0;
		gyro.up[i] = 0;
	}
	gyro.forward[X_AXIS] = FAC_IMU_UNIT_SCALE;
	gyro.left[Y_AXIS] = FAC_IMU_UNIT_SCALE;
	gyro.up[Z_AXIS] = FAC_IMU_UNIT_SCALE;
	gyro.accel_level.is_valid = FALSE;
	gyro.gyro_level.is_valid = FALSE;
}

/*
 * @brief	Component of a body vector along one axis of the calibrated frame
 * @note	The widest term is 1000 * 32767, and three of them summed stay under 10^8
 * @retval	the dot product brought back to the scale of the body vector, 1 division
 */
static int32_t FAC_IMU_project(const int16_t world_axis[LSM6DS3_AXIS_NUMBER], const int32_t body[LSM6DS3_AXIS_NUMBER]) {
	int32_t dot = 0;
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		dot += (int32_t) world_axis[i] * body[i];
	return dot / FAC_IMU_UNIT_SCALE;
}

/*
 * @brief	Length of a vector
 * @IMPORTANT	Every component must already be inside +-IMU_CALIBRATION_ACCEL_LIMIT or the sum of
 * 				the squares overflows: three times 32767 squared is 3.2*10^9 and int32_t stops at
 * 				2.1*10^9. The check belongs BEFORE the squaring, never after it
 * @retval	the exact integer square root of the sum of the squares, no division
 */
static int32_t FAC_IMU_norm(const int32_t vector[LSM6DS3_AXIS_NUMBER]) {
	int32_t sum_of_squares = 0;
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		sum_of_squares += vector[i] * vector[i];
	return FAC_math_sqrt(sum_of_squares);
}

/*
 * @brief	Turn a vector into a unit one on the FAC_IMU_UNIT_SCALE scale, in place
 * @retval	FALSE when the vector has no length and therefore no direction to keep
 */
static uint8_t FAC_IMU_normalize(int32_t vector[LSM6DS3_AXIS_NUMBER]) {
	int32_t length = FAC_IMU_norm(vector);
	if (length == 0)
		return FALSE;
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		vector[i] = (vector[i] * FAC_IMU_UNIT_SCALE) / length;	// 3 divisions, paid once per calibration
	return TRUE;
}

/*
 * @brief	Decide whether a still measure is trustworthy enough to calibrate from
 * @note	Three independent questions. The spans say the robot did not rotate and was not being
 * 			carried, the magnitude says the accelerometer was reading gravity alone and not gravity
 * 			plus somebody accelerating it, and the per component limit is the overflow guard the
 * 			magnitude itself needs
 * @note	What CANNOT be checked here, and is not attempted anywhere, is an accelerometer bias:
 * 			from a single pose a bias and a tilt are the same reading. The calibrated frame is what
 * 			replaces it, since it removes the apparent offset a tilted mounting produces
 * @retval	TRUE when every check passed
 */
static uint8_t FAC_IMU_VALIDATE_still_measure(const LSM6DS3_still_measure *measure) {
	int32_t accel[LSM6DS3_AXIS_NUMBER];
	int32_t magnitude;

	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++) {
		if (measure->gyro_span[i] > IMU_CALIBRATION_MAX_GYRO_SPAN)
			return FALSE;	// the robot rotated inside the window
		if (measure->accel_span[i] > IMU_CALIBRATION_MAX_ACCEL_SPAN)
			return FALSE;	// the robot was moved inside the window
		accel[i] = measure->accel_mean[i];
		if (accel[i] > IMU_CALIBRATION_ACCEL_LIMIT || accel[i] < -IMU_CALIBRATION_ACCEL_LIMIT)
			return FALSE;	// checked BEFORE squaring, see FAC_IMU_norm
	}

	magnitude = FAC_IMU_norm(accel);
	if (magnitude < IMU_CALIBRATION_ACCEL_MIN || magnitude > IMU_CALIBRATION_ACCEL_MAX)
		return FALSE;	// not one g: it is not simply standing on a floor

	return TRUE;
}

/*
 * @brief	Build the world frame out of the measured direction of gravity
 * @note	Right handed, z up: x is forward, y is left, z is the real vertical. Only the vertical
 * 			is measured. Two of the three attitude degrees of freedom are all a stationary
 * 			accelerometer can fix, roll and pitch: turning about the vertical does not change the
 * 			reading by one count, and there is no magnetometer on this board. So the yaw origin is
 * 			arbitrary by construction, it is simply "where the front of the board pointed when the
 * 			robot was armed", and it is stable for the whole run. It is not a compass
 * @note	The degenerate case is real, not theoretical: a FAC mounted on its side has its own +x
 * 			nearly vertical, and projecting +x on the horizontal plane then collapses to nothing.
 * 			Body +y becomes the reference in that case
 * @note	Nothing is written into the module state until every step succeeded, so a failure here
 * 			leaves the previous frame exactly as it was
 * @retval	FALSE if the geometry degenerated anyway, and in that case the frame is unchanged
 */
static uint8_t FAC_IMU_BUILD_frame(const int16_t accel_mean[LSM6DS3_AXIS_NUMBER]) {
	int32_t up[LSM6DS3_AXIS_NUMBER];
	int32_t forward[LSM6DS3_AXIS_NUMBER];
	int32_t left[LSM6DS3_AXIS_NUMBER];
	int32_t reference[LSM6DS3_AXIS_NUMBER] = { 0, 0, 0 };
	int32_t dot = 0;

	/* UP IS WHERE THE ACCELEROMETER POINTS WHILE THE ROBOT IS STILL */
	// a mems accelerometer measures specific force, so at rest the axis pointing upwards is the
	// one reading +1g: the vector arrives here already pointing up and is NOT negated.
	// DO NOT NEGATE IT HERE if a board ever needs the opposite convention, use IMU_ACCEL_SIGN in
	// config.h instead. The reason is that the sign appears TWICE on the accelerometer path and
	// cancels: a sensor reporting -s would also make this store up = -u, and the projection would
	// then give (-u).(-s) = u.s. Negating up alone breaks that pairing and leaves a single
	// negation, so the levelled z comes out inverted and is_upside_down reads true while the robot
	// is the right way up. IMU_ACCEL_SIGN sits at the source instead, where it restores both.
	// Worth knowing what an unfixed wrong sign would actually cost, because it is not what one
	// expects: forward is invariant too (its (X.up)*up term holds up twice), so tilt,
	// is_upside_down and the levelled accelerometer y and z all still come out right, and only up
	// and left end up negated, which is this same frame turned 180 degrees about forward. The
	// GYROSCOPE is what pays, since an accelerometer convention does not negate it and there is no
	// second sign to cancel against: FAC_IMU_GET_yaw_rate_mdps, and the heading built on it, run
	// backwards
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		up[i] = accel_mean[i];
	if (!FAC_IMU_normalize(up))
		return FALSE;

	/* FORWARD IS BODY +X PROJECTED ON THE HORIZONTAL PLANE */
	if (up[X_AXIS] > IMU_CALIBRATION_VERTICAL_LIMIT || up[X_AXIS] < -IMU_CALIBRATION_VERTICAL_LIMIT)
		reference[Y_AXIS] = FAC_IMU_UNIT_SCALE;	// body +x is nearly vertical, it cannot define a forward
	else
		reference[X_AXIS] = FAC_IMU_UNIT_SCALE;

	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		dot += reference[i] * up[i];
	dot /= FAC_IMU_UNIT_SCALE;
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		forward[i] = reference[i] - (dot * up[i]) / FAC_IMU_UNIT_SCALE;
	if (!FAC_IMU_normalize(forward))
		return FALSE;

	/* LEFT COMPLETES THE RIGHT HANDED TRIPLE */
	// check it on the trivial case: forward = (1000, 0, 0) and up = (0, 0, 1000) give (0, 1000, 0)
	left[X_AXIS] = (up[Y_AXIS] * forward[Z_AXIS] - up[Z_AXIS] * forward[Y_AXIS]) / FAC_IMU_UNIT_SCALE;
	left[Y_AXIS] = (up[Z_AXIS] * forward[X_AXIS] - up[X_AXIS] * forward[Z_AXIS]) / FAC_IMU_UNIT_SCALE;
	left[Z_AXIS] = (up[X_AXIS] * forward[Y_AXIS] - up[Y_AXIS] * forward[X_AXIS]) / FAC_IMU_UNIT_SCALE;

	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++) {
		gyro.up[i] = (int16_t) up[i];
		gyro.forward[i] = (int16_t) forward[i];
		gyro.left[i] = (int16_t) left[i];
	}
	gyro.accel_level.is_valid = FALSE;	// whatever was cached was projected on the old frame
	gyro.gyro_level.is_valid = FALSE;
	return TRUE;
}

/*
 * @brief	The levelled accelerometer, recomputed only when the sensor was read again
 * @note	This is where the laziness of the whole feature lives. FAC_IMU_update stays one burst
 * 			read and three guards, and the projection is paid here, at most once per sensor read
 * 			and only by a robot whose mix actually asks for levelled data
 */
static const FAC_IMU_level_cache* FAC_IMU_GET_accel_level(void) {
	if (!gyro.accel_level.is_valid || gyro.accel_level.last_update_tick != gyro.last_update_tick) {
		int32_t body[LSM6DS3_AXIS_NUMBER];
		for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
			body[i] = FAC_IMU_GET_accel_raw(i);	// through the accessor, so IMU_ACCEL_SIGN applies here too
		gyro.accel_level.value[X_AXIS] = FAC_IMU_project(gyro.forward, body);
		gyro.accel_level.value[Y_AXIS] = FAC_IMU_project(gyro.left, body);
		gyro.accel_level.value[Z_AXIS] = FAC_IMU_project(gyro.up, body);
		gyro.accel_level.last_update_tick = gyro.last_update_tick;
		gyro.accel_level.is_valid = TRUE;
	}
	return &gyro.accel_level;
}

/*
 * @brief	The levelled gyroscope, cached the same way as the accelerometer one
 * @note	Two separate caches and not one: a mix asking only for the yaw rate must not pay the
 * 			projection of the accelerometer as well
 */
static const FAC_IMU_level_cache* FAC_IMU_GET_gyro_level(void) {
	if (!gyro.gyro_level.is_valid || gyro.gyro_level.last_update_tick != gyro.last_update_tick) {
		int32_t body[LSM6DS3_AXIS_NUMBER];
		for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
			body[i] = FAC_IMU_GET_LSM6DS3_object()->gyro[i];
		gyro.gyro_level.value[X_AXIS] = FAC_IMU_project(gyro.forward, body);
		gyro.gyro_level.value[Y_AXIS] = FAC_IMU_project(gyro.left, body);
		gyro.gyro_level.value[Z_AXIS] = FAC_IMU_project(gyro.up, body);
		gyro.gyro_level.last_update_tick = gyro.last_update_tick;
		gyro.gyro_level.is_valid = TRUE;
	}
	return &gyro.gyro_level;
}

/*
 * @brief	The accumulator converted into a binary angle
 * @note	Every intermediate stays inside int32_t: the accumulator divided by 1000 is at most
 * 			359999, and multiplied by 4096 that is 1.47*10^9
 * @retval	the heading in [0, 4095], 2 divisions
 */
static int32_t FAC_IMU_GET_heading_from_accumulator(void) {
	return ((gyro.heading_udeg / 1000) * FAC_ANGLE_TURN) / 360000;
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

/*
 * @brief	Acceleration of one axis in raw counts, on the standard convention
 * @note	IMU_ACCEL_SIGN is applied here, which is the single point the whole firmware depends on:
 * 			every other consumer, the level frame and the telemetry included, goes through this
 * 			accessor and therefore always sees an axis pointing upwards as reading +1g
 * @note	The sign is a compile time +-1, so at +1 the multiplication disappears and the clamp
 * 			with it (the compiler can prove an int16_t times one still fits). It only becomes real
 * 			code on a board that needs the opposite convention, where the clamp is what stops the
 * 			negation of the negative full scale from wrapping
 */
int16_t FAC_IMU_GET_accel_raw(uint8_t axis) {
	if (axis >= LSM6DS3_AXIS_NUMBER)
		return 0;
	return (int16_t) FAC_math_clamp_to(
			(int32_t) FAC_IMU_GET_LSM6DS3_object()->accel[axis] * IMU_ACCEL_SIGN,
			INT16_MIN, INT16_MAX);
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
 * @brief		Zero the gyroscope and measure which way is really up
 * @note		This is NOT a boot time call. At power up the robot may still be in the hand of its
 * 				operator, and offsets measured on a moving body are wrong for the whole run. It
 * 				belongs to the moment the robot is armed, when it is presumed placed in the arena
 * 				and standing still
 * @note		Blocks for about 400ms and refreshes the watchdog through the driver. One attempt
 * 				per call and no retry loop inside: retrying is the caller business, because between
 * 				two attempts the main loop has to keep running
 * @note		On success it also resets the heading, whose origin is the front of the board at the
 * 				instant of this calibration and means nothing across two different ones
 * @IMPORTANT	On ANY failure nothing changes: the previous offsets are put back and the previous
 * 				frame is left alone. That is what makes a re-arm in the middle of a match safe, an
 * 				attempt made while the robot is moving or upside down cannot destroy a good
 * 				calibration it already has
 * @retval		TRUE when the measurement passed validation and was applied
 */
uint8_t FAC_IMU_calibrate(void) {
	LSM6DS3_still_measure measure;
	int16_t previous_offsets[LSM6DS3_AXIS_NUMBER];
	int16_t offsets[LSM6DS3_AXIS_NUMBER];
	int16_t accel_mean[LSM6DS3_AXIS_NUMBER];

	if (!gyro.is_initialized || FAC_IMU_GET_status() == HAL_ERROR) {
		return FALSE;	// the sensor is missing or down, there is nothing to calibrate
	}

	// the measure zeroes the offsets to read the bare sensor, so keep a copy to put back on failure
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		previous_offsets[i] = FAC_IMU_GET_LSM6DS3_object()->gyro_offsets[i];

	if (LSM6DS3_measure_still(FAC_IMU_GET_LSM6DS3_object(), &measure) != HAL_OK) {
		FAC_IMU_SET_status(HAL_ERROR);
		LSM6DS3_SET_gyro_offsets(FAC_IMU_GET_LSM6DS3_object(), previous_offsets);
		return FALSE;
	}
	if (!FAC_IMU_VALIDATE_still_measure(&measure)) {
		LSM6DS3_SET_gyro_offsets(FAC_IMU_GET_LSM6DS3_object(), previous_offsets);
		return FALSE;
	}

	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		offsets[i] = (int16_t) FAC_math_clamp_to(-(int32_t) measure.gyro_mean[i], INT16_MIN, INT16_MAX);

	// the measure comes straight from the driver, which knows nothing of the board convention, so
	// IMU_ACCEL_SIGN is applied here the same way FAC_IMU_GET_accel_raw applies it to a live reading
	for (int i = 0; i < LSM6DS3_AXIS_NUMBER; i++)
		accel_mean[i] = (int16_t) FAC_math_clamp_to(
				(int32_t) measure.accel_mean[i] * IMU_ACCEL_SIGN, INT16_MIN, INT16_MAX);

	if (!FAC_IMU_BUILD_frame(accel_mean)) {
		LSM6DS3_SET_gyro_offsets(FAC_IMU_GET_LSM6DS3_object(), previous_offsets);
		return FALSE;
	}

	LSM6DS3_SET_gyro_offsets(FAC_IMU_GET_LSM6DS3_object(), offsets);
	gyro.is_calibrated = TRUE;
	FAC_IMU_RESET_heading();
	return TRUE;
}

uint8_t FAC_IMU_GET_is_calibrated(void) {
	return gyro.is_calibrated;
}

/*
 * @brief		Acceleration of one axis of the CALIBRATED frame, in raw counts
 * @note		Call FAC_IMU_update first, exactly as for the body accessors: this reprojects what
 * 				that read brought home, it does not touch the bus itself
 * @note		int32_t and not int16_t on purpose: a projection keeps the length of the vector,
 * 				which can reach 32767 times the square root of three and does not fit an int16_t
 * @IMPORTANT	Meaningless unless FAC_IMU_GET_is_calibrated is TRUE. Without a calibration the
 * 				frame is the identity and these return the board own axis, which is a defined and
 * 				harmless fallback but is not levelled
 * @retval		the axis in raw accelerometer counts, 0 for an axis above Z_AXIS
 */
int32_t FAC_IMU_GET_accel_level_raw(uint8_t axis) {
	if (axis >= LSM6DS3_AXIS_NUMBER)
		return 0;
	return FAC_IMU_GET_accel_level()->value[axis];
}

/*
 * @brief	Angular rate of one axis of the CALIBRATED frame, in raw counts
 * @note	Same contract as FAC_IMU_GET_accel_level_raw
 */
int32_t FAC_IMU_GET_gyro_level_raw(uint8_t axis) {
	if (axis >= LSM6DS3_AXIS_NUMBER)
		return 0;
	return FAC_IMU_GET_gyro_level()->value[axis];
}

/*
 * @brief	One component of the measured vertical, expressed in BODY coordinates
 * @retval	the component on the FAC_IMU_UNIT_SCALE scale, so (0, 0, 1000) is a board mounted flat
 */
int16_t FAC_IMU_GET_up(uint8_t axis) {
	if (axis >= LSM6DS3_AXIS_NUMBER)
		return 0;
	return gyro.up[axis];
}

/*
 * @brief	How far the robot has tilted away from the attitude it was armed in
 * @note	The angle between the current specific force and the calibrated vertical, taken from
 * 			the levelled reading: the horizontal part against the vertical one
 * @note	It is an accelerometer angle, so it is only meaningful while the robot is not being
 * 			accelerated hard. During an impact it says nothing
 * @retval	a binary angle in [0, 2048]: 0 is the attitude it was armed in, FAC_ANGLE_QUARTER_TURN
 * 			is on its side, FAC_ANGLE_HALF_TURN is upside down
 */
int32_t FAC_IMU_GET_tilt_angle(void) {
	const FAC_IMU_level_cache *level = FAC_IMU_GET_accel_level();
	int32_t x = FAC_math_clamp_to(level->value[X_AXIS], -TILT_COMPONENT_LIMIT, TILT_COMPONENT_LIMIT);
	int32_t y = FAC_math_clamp_to(level->value[Y_AXIS], -TILT_COMPONENT_LIMIT, TILT_COMPONENT_LIMIT);
	int32_t z = FAC_math_clamp_to(level->value[Z_AXIS], -TILT_COMPONENT_LIMIT, TILT_COMPONENT_LIMIT);
	int32_t horizontal = FAC_math_sqrt(x * x + y * y);	// the clamp above is what keeps this sum inside int32_t
	return FAC_math_atan2(horizontal, z);	// horizontal is never negative, so the result stays in the first two quadrants
}

/*
 * @brief	TRUE when the robot is past ninety degrees from the attitude it was armed in
 * @note	The sign of the levelled vertical component IS that test, exactly: the tilt angle
 * 			passes FAC_ANGLE_QUARTER_TURN precisely when that component goes negative. So this
 * 			costs one projection and no trigonometry at all, while FAC_IMU_GET_tilt_angle would
 * 			pay a square root and six divisions to answer the same question
 * @note	No hysteresis here. A mix that switches something on this must add its own, or it will
 * 			chatter while the robot balances on an edge
 */
uint8_t FAC_IMU_GET_is_upside_down(void) {
	return (FAC_IMU_GET_accel_level()->value[Z_AXIS] < 0) ? TRUE : FALSE;
}

/*
 * @brief	Rate of rotation about the CALIBRATED vertical, in milli degrees per second
 * @note	This is the concrete win of the calibration: the real yaw rate whatever the mounting
 * 			angle of the board, instead of a gyroscope z that is wrong the moment the FAC is not
 * 			mounted perfectly flat
 * @note	Positive is counterclockwise seen from above, the same direction FAC_math_atan2 grows in
 * @retval	the yaw rate in mdps, and it can exceed the +-2000000 of a single axis full scale only
 * 			by the projection of two saturated axis, still far inside int32_t
 */
int32_t FAC_IMU_GET_yaw_rate_mdps(void) {
	return FAC_IMU_GET_gyro_level()->value[Z_AXIS] * LSM6DS3_GYRO_SENSITIVITY_MDPS;
}

/*
 * @brief		Heading about the calibrated vertical, integrated from the yaw rate
 * @note		THE INTEGRATOR LIVES HERE, not in the main loop. Every call advances it by the time
 * 				since the previous one, so a mix calling it once per pass gets a correct integral
 * 				and a firmware where nobody calls it pays exactly nothing
 * @note		Call FAC_IMU_update before it, or it integrates a rate that is arbitrarily old
 * @note		UNITS. The accumulator is in micro degrees because a rate in mdps multiplied by a
 * 				delta in ms lands there with no scaling: mdps * ms is micro degrees exactly. It is
 * 				kept inside [0, 360000000) by conditional add and subtract instead of a modulo,
 * 				since a modulo is a library call on a core with no divider. The wrap is a loop and
 * 				not a single test on purpose: at the yaw rates this sensor can report, one step
 * 				across the whole FAC_IMU_HEADING_MAX_GAP window can be worth more than a full turn
 * @IMPORTANT	IT DRIFTS, and it is relative to the last reset, never to any absolute direction.
 * 				There is no magnetometer on this board and gravity says nothing about yaw, so an
 * 				absolute heading is not merely unimplemented, it is unobservable. With a good
 * 				calibration expect a residual of the order of 0.5 to 2 degrees per second
 * @note		Three cases resynchronise the timestamp instead of integrating, because a single
 * 				rate sample says nothing about the interval it would be multiplied by: the first
 * 				call, a call while the sensor is in error, and a gap longer than
 * 				FAC_IMU_HEADING_MAX_GAP. That last one is a real hole and not a theoretical one,
 * 				the whole mix chain stops running while the robot is disarmed
 * @retval		a binary angle in [0, 4095], ready for FAC_math_sin and FAC_math_cos
 */
int32_t FAC_IMU_GET_heading(void) {
	uint32_t now = HAL_GetTick();
	uint32_t elapsed = now - gyro.heading_tick;

	if (!gyro.is_heading_synced || FAC_IMU_GET_status() == HAL_ERROR
			|| elapsed > FAC_IMU_HEADING_MAX_GAP) {
		gyro.heading_tick = now;
		gyro.is_heading_synced = TRUE;
		return FAC_IMU_GET_heading_from_accumulator();
	}
	if (elapsed == 0) {
		return FAC_IMU_GET_heading_from_accumulator();	// nothing to integrate over, and the rate has not changed either
	}

	gyro.heading_tick = now;
	gyro.heading_udeg += FAC_IMU_GET_yaw_rate_mdps() * (int32_t) elapsed;
	while (gyro.heading_udeg >= HEADING_FULL_TURN_UDEG)
		gyro.heading_udeg -= HEADING_FULL_TURN_UDEG;
	while (gyro.heading_udeg < 0)
		gyro.heading_udeg += HEADING_FULL_TURN_UDEG;

	return FAC_IMU_GET_heading_from_accumulator();
}

/*
 * @brief	Put the heading back to zero and resynchronise its clock
 * @note	This is what a heading hold mix calls whenever the pilot commands a turn: the heading
 * 			is a relative angle, so "straight ahead" is wherever it was last reset
 */
void FAC_IMU_RESET_heading(void) {
	gyro.heading_udeg = 0;
	gyro.heading_tick = HAL_GetTick();
	gyro.is_heading_synced = TRUE;
}

/*
 * @brief		Initialize the gyro/accel LSM6DS3
 * @note		Identification and configuration only, no calibration: at boot the robot may still
 * 				be in a hand, see FAC_IMU_calibrate
 * @IMPORTANT	If status is HAL_ERROR the mix or special function must manage this problem
 */
HAL_StatusTypeDef FAC_IMU_init(void) {
	HAL_StatusTypeDef gyro_status;
	HAL_Delay(200);	// wait for the imu startup
	gyro.is_initialized = FALSE;
	gyro.last_update_tick = 0;
	gyro.is_calibrated = FALSE;
	FAC_IMU_SET_identity_frame();
	FAC_IMU_RESET_heading();
	gyro_status = LSM6DS3_init(FAC_IMU_GET_LSM6DS3_object(), &hi2c1);
	FAC_IMU_SET_status(gyro_status);	// on success too: it used to be set only on failure, and the good case worked only because the struct is zero initialized and HAL_OK is 0
	return gyro_status;
}
