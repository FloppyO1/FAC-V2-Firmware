/*
 * fac_math.h
 *
 *	MATH PRIMITIVES FOR MIXES AND SPECIAL FUNCTIONS
 *
 *	Everything a mix or a special function needs to compute, without ever using a float.
 *	The STM32F072 is a Cortex-M0, which has no floating point unit, so every float operation is
 *	a library call of hundreds of clock cycles. All the math here is integer instead.
 *
 *	WHAT COSTS WHAT
 *	The Cortex-M0 has no hardware divider either, and no long multiply (umull), which is what a
 *	compiler would need to replace a division by a constant with a multiply and a shift. So even
 *	a plain "/ 1000" becomes a call to __aeabi_idiv. Integer division is therefore the expensive
 *	operation here (still several times cheaper than the float it replaces), and it is worth
 *	knowing which primitive pays for one:
 *		0 divisions : clamp, abs, add, sub, min, max, clamp_to, angle_wrap
 *		1 division  : mul, scale, blend, deadzone, to_range, from_range, mul_scaled, div_scaled
 *		3 divisions : expo
 *		4 divisions : sin, cos
 *		6 divisions : atan2
 *	A mix built out of add/sub/clamp costs no division at all, which covers most of them.
 *	sqrt is the one exception to reading this list as a cost: it pays no division, but it always
 *	walks 16 shift-and-subtract steps, so budget it like a couple of divisions rather than as free.
 *
 *	THE VALUE SCALE
 *	A normalized value is an int32_t in [-1000, +1000], where 1000 means "full scale".
 *	It is the format of every mix/function input and output. It is not an arbitrary choice: it
 *	matches RECEIVER_CHANNEL_RESOLUTION, MOTOR_SPEED_RESOLUTION and SERVO_POSITION_RESOLUTION,
 *	so the conversions at both ends of the chain are exact and nothing is rounded away.
 *
 *	THE THREE GROUPS
 *	1) NORMALIZED VALUES  - work on [-1000, +1000] and SATURATE, they never wrap around.
 *	                        This is what a mix uses for almost everything.
 *	2) RAW FIXED POINT    - work on the full int32_t range with an explicit scale and DO NOT
 *	                        saturate. For processing sensor data (gyroscope, accelerometer),
 *	                        where saturating at 1000 would throw the signal away.
 *	3) ANGLES AND TRIGONOMETRY - for anything that spins, melty brain first of all.
 *
 *	EXACTNESS RULES (a PC simulator of a mix must reproduce these to get identical results)
 *	- Integer division TRUNCATES TOWARD ZERO: -7/2 is -3, not -4. In javascript that is
 *	  Math.trunc(), NOT Math.floor(). This is the classic difference that diverges silently.
 *	- Group 1 always clamps its result to [-1000, +1000].
 *	- Every intermediate fits in int32_t. Where a function has an input range limit to respect,
 *	  it is written in its @IMPORTANT line.
 *
 *  Created on: Aug 1, 2026
 *      Author: Filippo
 */

#ifndef INC_FAC_CODE_MIXES_FUNCTIONS_FAC_MATH_H_
#define INC_FAC_CODE_MIXES_FUNCTIONS_FAC_MATH_H_

#include "stm32f0xx_hal.h"

/* THE NORMALIZED VALUE TYPE */
// int32_t and not int16_t on purpose: the Cortex-M0 works on 32 bit registers, so a 16 bit type
// costs an extra sign extension instruction on almost every operation
typedef int32_t fac_value_t;

#define FAC_VALUE_MAX 1000		// full scale, forward for a motor / full travel for a servo
#define FAC_VALUE_MIN (-1000)	// full scale in the opposite direction
#define FAC_VALUE_ZERO 0		// motor stopped / servo centered

/* THE ANGLE UNIT */
// a full turn is 4096 units (binary angle): wrapping is a single AND instead of a modulo, which
// on a mcu without a divider would be a library call. One unit is 360/4096 = 0.088 degrees
#define FAC_ANGLE_TURN 4096
#define FAC_ANGLE_MASK (FAC_ANGLE_TURN - 1)
#define FAC_ANGLE_HALF_TURN (FAC_ANGLE_TURN / 2)	// 180 degrees
#define FAC_ANGLE_QUARTER_TURN (FAC_ANGLE_TURN / 4)	// 90 degrees
// write angles in degrees and let this convert them: FAC_math_sin(FAC_MATH_DEG(90))
#define FAC_MATH_DEG(d) (((int32_t) (d) * FAC_ANGLE_TURN) / 360)

// the trigonometric functions work ten times finer than the value scale internally, and only
// come back to it at the end: it halves the truncation error for free, the operations are the same
#define FAC_MATH_FINE (FAC_VALUE_MAX * 10)

/* ------------------------------------------------------------------------------------------ */
/* 1) NORMALIZED VALUES                                                                       */
/* ------------------------------------------------------------------------------------------ */

/*
 * @brief	Keep a value inside the normalized range
 * @retval	v limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_clamp(fac_value_t v) {
	if (v > FAC_VALUE_MAX)
		return FAC_VALUE_MAX;
	if (v < FAC_VALUE_MIN)
		return FAC_VALUE_MIN;
	return v;
}

/*
 * @brief	Absolute value, limited to the normalized range
 * @retval	how far the value is from zero, [0, +1000]
 */
static inline fac_value_t FAC_math_abs(fac_value_t v) {
	v = FAC_math_clamp(v);
	return (v < 0) ? -v : v;
}

/*
 * @brief	Sum of two normalized values, saturated
 * @note	This is what replaces the manual range juggling a mix would otherwise need: full
 * 			throttle plus full steering gives full scale, not an overflowed value
 * @retval	a + b limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_add(fac_value_t a, fac_value_t b) {
	return FAC_math_clamp(FAC_math_clamp(a) + FAC_math_clamp(b));
}

/*
 * @brief	Difference of two normalized values, saturated
 * @retval	a - b limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_sub(fac_value_t a, fac_value_t b) {
	return FAC_math_clamp(FAC_math_clamp(a) - FAC_math_clamp(b));
}

/*
 * @brief	Product of two normalized values
 * @note	Both are fractions of full scale, so the product is a*b/1000 and not a*b: use it to
 * 			modulate a value with another one, for example a throttle scaled by a knob channel
 * @retval	a*b/1000, so 1000 behaves as "times one" and 500 as "half"
 */
static inline fac_value_t FAC_math_mul(fac_value_t a, fac_value_t b) {
	return FAC_math_clamp(
			(FAC_math_clamp(a) * FAC_math_clamp(b)) / FAC_VALUE_MAX);
}

/*
 * @brief	Scale a normalized value by a percentage
 * @note	percent is a plain percentage: 100 leaves the value alone, 50 halves it, 200 doubles
 * 			it (and then saturates), a negative one also reverses it
 * @IMPORTANT	percent is limited to +-10000 to keep the product inside int32_t
 * @retval	v * percent / 100, limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_scale(fac_value_t v, int32_t percent) {
	if (percent > 10000)
		percent = 10000;
	if (percent < -10000)
		percent = -10000;
	return FAC_math_clamp((FAC_math_clamp(v) * percent) / 100);
}

/*
 * @brief	The smaller of two normalized values
 */
static inline fac_value_t FAC_math_min(fac_value_t a, fac_value_t b) {
	a = FAC_math_clamp(a);
	b = FAC_math_clamp(b);
	return (a < b) ? a : b;
}

/*
 * @brief	The bigger of two normalized values
 */
static inline fac_value_t FAC_math_max(fac_value_t a, fac_value_t b) {
	a = FAC_math_clamp(a);
	b = FAC_math_clamp(b);
	return (a > b) ? a : b;
}

/*
 * @brief	Linear blend between two normalized values
 * @note	weight goes from 0 (all a) to 1000 (all b), 500 is the average of the two. Useful to
 * 			fade between two behaviours with a channel instead of switching abruptly
 * @retval	the blended value, limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_blend(fac_value_t a, fac_value_t b,
		fac_value_t weight) {
	if (weight <= 0)
		return FAC_math_clamp(a);
	if (weight >= FAC_VALUE_MAX)
		return FAC_math_clamp(b);
	return FAC_math_clamp(
			(FAC_math_clamp(a) * (FAC_VALUE_MAX - weight)
					+ FAC_math_clamp(b) * weight) / FAC_VALUE_MAX);
}

/*
 * @brief	Ignore the values close to zero and use the whole output range for the rest
 * @note	size is how wide the dead band is, in normalized units: 50 means "ignore the first
 * 			5% of stick travel". Outside the band the value is stretched back to full scale, so
 * 			the stick still reaches +-1000 at its ends
 * @note	The receiver already applies its own deadzone to the channels, this one is for a mix
 * 			that needs a different band on a value it computed itself
 * @retval	the shaped value, limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_deadzone(fac_value_t v, fac_value_t size) {
	if (size <= 0)
		return FAC_math_clamp(v);
	if (size >= FAC_VALUE_MAX)
		return FAC_VALUE_ZERO;// the band covers everything, nothing is left to output

	fac_value_t magnitude = FAC_math_abs(v);
	if (magnitude <= size)
		return FAC_VALUE_ZERO;	// inside the dead band

	fac_value_t out = ((magnitude - size) * FAC_VALUE_MAX)
			/ (FAC_VALUE_MAX - size);
	return (v < 0) ? -out : out;
}

/*
 * @brief	Soften the response around the center, keeping the full scale at the ends
 * @note	amount goes from 0 (perfectly linear) to 1000 (fully cubic). It is the classic RC
 * 			expo: fine control around the center, full power still available at the ends
 * @retval	the shaped value, limited to [-1000, +1000]
 */
static inline fac_value_t FAC_math_expo(fac_value_t v, fac_value_t amount) {
	v = FAC_math_clamp(v);
	if (amount <= 0)
		return v;
	if (amount > FAC_VALUE_MAX)
		amount = FAC_VALUE_MAX;

	// v^3 in normalized units, computed in two steps to keep every intermediate around 10^6
	int32_t cube = (((v * v) / FAC_VALUE_MAX) * v) / FAC_VALUE_MAX;
	return FAC_math_clamp(
			(v * (FAC_VALUE_MAX - amount) + cube * amount) / FAC_VALUE_MAX);
}

/*
 * @brief	Convert a normalized value into an arbitrary range
 * @note	For driving something that has its own units, for example a servo pulse in us
 * @IMPORTANT	(out_max - out_min) must stay below about 10^6 to keep the product in int32_t
 * @retval	-1000 maps to out_min and +1000 maps to out_max
 */
static inline int32_t FAC_math_to_range(fac_value_t v, int32_t out_min,
		int32_t out_max) {
	v = FAC_math_clamp(v);
	return ((v - FAC_VALUE_MIN) * (out_max - out_min))
			/ (FAC_VALUE_MAX - FAC_VALUE_MIN) + out_min;
}

/*
 * @brief	Convert a value of an arbitrary range into a normalized one
 * @note	The entry point for anything that is not already normalized, a raw sensor reading
 * 			for instance. x is clamped into [in_min, in_max] first
 * @IMPORTANT	(in_max - in_min) must stay below about 10^6 to keep the product in int32_t
 * @retval	in_min maps to -1000 and in_max maps to +1000, 0 if the input range is empty
 */
static inline fac_value_t FAC_math_from_range(int32_t x, int32_t in_min,
		int32_t in_max) {
	if (in_max == in_min)
		return FAC_VALUE_ZERO;// empty input range, there is nothing to divide by
	if (x > in_max)
		x = in_max;
	if (x < in_min)
		x = in_min;
	return ((x - in_min) * (FAC_VALUE_MAX - FAC_VALUE_MIN)) / (in_max - in_min)
			+ FAC_VALUE_MIN;
}

/* ------------------------------------------------------------------------------------------ */
/* 2) RAW FIXED POINT                                                                         */
/* ------------------------------------------------------------------------------------------ */
// These work on the whole int32_t range and DO NOT saturate at 1000. They are the tools for
// processing sensor data: a gyroscope reading is +-32768 raw, and squeezing it into +-1000
// before doing the math would throw the signal away. Normalize at the END, when the result
// becomes a mix output, with FAC_math_from_range.
/*
 * @brief	Multiply two fixed point numbers sharing the same scale
 * @note	scale is what represents "one": with scale 1000, a and b are thousandths
 * @IMPORTANT	a * b must fit in int32_t, so keep both below about 46000 when they are similar
 * @retval	a * b / scale, 0 if scale is 0
 */
static inline int32_t FAC_math_mul_scaled(int32_t a, int32_t b, int32_t scale) {
	if (scale == 0)
		return 0;
	return (a * b) / scale;
}

/*
 * @brief	Divide two fixed point numbers keeping the result on the same scale
 * @IMPORTANT	a * scale must fit in int32_t
 * @retval	a * scale / b, 0 if b is 0
 */
static inline int32_t FAC_math_div_scaled(int32_t a, int32_t b, int32_t scale) {
	if (b == 0)
		return 0;
	return (a * scale) / b;
}

/*
 * @brief	Keep a raw value inside explicit limits
 * @note	The raw counterpart of FAC_math_clamp, which is fixed on [-1000, +1000]
 * @retval	v limited to [min, max]
 */
static inline int32_t FAC_math_clamp_to(int32_t v, int32_t min, int32_t max) {
	if (v > max)
		return max;
	if (v < min)
		return min;
	return v;
}

/*
 * @brief	Integer square root
 * @note	EXACT, not an approximation: it returns the largest r with r*r <= v, so it is the real
 * 			square root truncated toward zero. The trigonometry above is approximated, this is not
 * @note	It costs no division, which is why it is written this way instead of with a Newton
 * 			iteration: the binary restoring method only shifts, compares and subtracts. It is not
 * 			free though, it always walks 16 steps, so budget it like a couple of divisions
 * @note	ON A SCALED VALUE the scale comes out halved, because sqrt(x*scale) is sqrt(x)*sqrt(scale).
 * 			To get the result back on the scale the input was in, multiply the input by that scale
 * 			once more before calling: FAC_math_sqrt(x * scale) is x's root, still on scale
 * @IMPORTANT	v must fit in int32_t AFTER any such pre-multiplication, so keep it under 2*10^9.
 * 			That is the real limit of the idiom above, and where a caller has to pick its units
 * @retval	the square root of v truncated toward zero, 0 for a negative v
 */
static inline int32_t FAC_math_sqrt(int32_t v) {
	if (v <= 0)
		return 0;
	uint32_t rest = (uint32_t) v;
	uint32_t root = 0;
	uint32_t bit = 1UL << 30;	// the highest power of four a positive int32_t can hold

	while (bit > rest)
		bit >>= 2;	// start from the first power of four that is not already too big

	while (bit != 0) {
		if (rest >= root + bit) {
			rest -= root + bit;
			root = (root >> 1) + bit;
		} else {
			root >>= 1;
		}
		bit >>= 2;
	}
	return (int32_t) root;
}

/* ------------------------------------------------------------------------------------------ */
/* 3) ANGLES AND TRIGONOMETRY                                                                 */
/* ------------------------------------------------------------------------------------------ */
// No lookup table: these use integer polynomial approximations, so they cost no flash for data
// and every division is by a constant. Accuracy measured against the real functions over the
// whole turn: sin and cos stay within 2 units out of 1000 (0.2% of full scale) and are exact at
// the four quadrant points, atan2 stays within 2 angle units (0.18 degrees).
// For scale: a robot spinning at 20 turns per second covers about 7 degrees between two
// iterations of the 1kHz main loop, so the loop rate limits the heading forty times more than
// these approximations do.
/*
 * @brief	Bring any angle back inside one turn
 * @note	A single AND, because a turn is a power of two. It works on negative angles too:
 * 			-512 comes back as 3584, which is the same direction
 * @retval	the angle in [0, 4095]
 */
static inline int32_t FAC_math_angle_wrap(int32_t angle) {
	return angle & FAC_ANGLE_MASK;
}

/*
 * @brief	Sine of an angle
 * @note	Parabolic approximation 4t(1-|t|) plus the classic 0.225 correction term, computed on
 * 			the fine scale and brought back to the value scale at the end. Measured error over the
 * 			whole turn: at most 2 units out of 1000, and exactly 0/+1000/0/-1000 at the quadrants
 * @retval	the sine as a normalized value, [-1000, +1000]
 */
static inline fac_value_t FAC_math_sin(int32_t angle) {
	int32_t t = FAC_math_angle_wrap(angle);
	if (t > FAC_ANGLE_HALF_TURN)
		t -= FAC_ANGLE_TURN;	// bring it to [-half turn, +half turn]
	t = (t * FAC_MATH_FINE) / FAC_ANGLE_HALF_TURN;// now +-10000 means +-180 degrees

	int32_t magnitude = (t < 0) ? -t : t;
	int32_t y = (4 * t * (FAC_MATH_FINE - magnitude)) / FAC_MATH_FINE;

	// correction term, it takes the error of the bare parabola from about 6% down to 0.2%
	int32_t yMagnitude = (y < 0) ? -y : y;
	y = y + (225 * ((y * yMagnitude) / FAC_MATH_FINE - y)) / 1000;

	return FAC_math_clamp(y / 10);// from the fine scale back to the value scale
}

/*
 * @brief	Cosine of an angle
 * @retval	the cosine as a normalized value, [-1000, +1000]
 */
static inline fac_value_t FAC_math_cos(int32_t angle) {
	return FAC_math_sin(angle + FAC_ANGLE_QUARTER_TURN);
}

/*
 * @brief	Arc tangent of a ratio between 0 and 1, given on the fine scale (10000 means 1.0)
 * @note	Helper of FAC_math_atan2, it only covers the first eighth of a turn
 * @retval	the angle in [0, 512], that is from 0 to 45 degrees
 */
static inline int32_t FAC_math_atan_ratio(int32_t ratio) {
	int32_t a = (ratio * (ratio - FAC_MATH_FINE)) / FAC_MATH_FINE;
	int32_t c = 1590 + (430 * ratio) / FAC_MATH_FINE;
	return (((FAC_ANGLE_TURN / 8) * ratio) / 100 - (a * c) / 1000) / 100;
}

/*
 * @brief	Angle of the vector (x, y), over the whole turn
 * @note	The way to get a direction out of two axes: the heading of a stick, or the tilt
 * 			direction read from two accelerometer axes. x and y only matter by their ratio, so
 * 			they can be raw sensor counts, they do not need to be normalized first
 * @note	Measured error against the real atan2 over 3600 directions: at most 2 angle units,
 * 			that is 0.18 degrees
 * @IMPORTANT	Keep |x| and |y| below about 200000: the ratio is computed as |y| * 10000 / |x|,
 * 				which has to stay inside int32_t. Any sensor of this board is well within it, a
 * 				raw accelerometer or gyroscope reading never leaves +-32768
 * @retval	the angle in [0, 4095], 0 pointing along +x and growing counterclockwise. Returns 0
 * 			when both x and y are 0, since there is no direction to report
 */
static inline int32_t FAC_math_atan2(int32_t y, int32_t x) {
	int32_t xMagnitude = (x < 0) ? -x : x;
	int32_t yMagnitude = (y < 0) ? -y : y;
	int32_t angle;

	if (xMagnitude == 0 && yMagnitude == 0)
		return 0;	// no direction at all

	if (xMagnitude >= yMagnitude) {	// first eighth of a turn, the ratio stays under one
		angle = FAC_math_atan_ratio((yMagnitude * FAC_MATH_FINE) / xMagnitude);
	} else {	// mirror it around 45 degrees to keep the ratio under one
		angle = FAC_ANGLE_QUARTER_TURN
				- FAC_math_atan_ratio(
						(xMagnitude * FAC_MATH_FINE) / yMagnitude);
	}

	if (x < 0)
		angle = FAC_ANGLE_HALF_TURN - angle;
	if (y < 0)
		angle = -angle;

	return FAC_math_angle_wrap(angle);
}

#endif /* INC_FAC_CODE_MIXES_FUNCTIONS_FAC_MATH_H_ */
