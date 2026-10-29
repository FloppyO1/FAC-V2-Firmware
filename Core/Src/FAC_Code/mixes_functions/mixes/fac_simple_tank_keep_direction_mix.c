/*
 * fac_simple_tank_keep_direction_mix.c	// 1) of HOW TO MAKE A MIX
 *
 *	HOW TO MAKE A MIX:
 *	1) Copy this file and the fac_template_mix.h.template and rename
 *		with the name of the new mix (ex: fac_simple_tank_mix.c/h)
 *		In the .h files, update the #ifndef and #define with the new name.
 *	2) Fix the include to match the new name you have choose
 *	3) Add to the FAC_MIXES_ID on fac_mixes.h file the new FAC_MIX
 *		(ex: FAC_MIX_SIMPLE_TANK) and set it to the mix_id
 *	4) Rename the FAC_<name>_mix_update() function on this file and
 *		on the fac_name_mix.h
 *	5) Add the FAC_<name>_mix_update() function into a new case into
 *		the fac_mixes.c file inside the FAC_mix_update() function using
 *		as case value your mix_id value.
 *	6) Add the include header file of your new mix inside the
 *		fac_mixes.c (#include "FAC_Code/mixes_functions/mixes/fac_<name>_mix.h")
 *	7) Make a description of the mix and describe all inputs/outputs it takes/gives
 *		Remeber: an input is a remote controller channel converted in
 *		standard values (the channel is chosen by the settings).
 *		If for example you need the gyroscope data you have to manage it
 *		inside your code.
 *	8) Include all the modules you need for your mix logic
 *		(uncomment the modules you need in CUSTOM MIX INCLUDE part)
 *	9) Write your mix logic code
 *
 *
 *  Created on: Oct 29, 2026
 *      Author: Floppy Lab
 */

#include "FAC_Code/mixes_functions/mixes/fac_simple_tank_keep_direction_mix.h"			// 2) of HOW TO MAKE A MIX

/* DEFAULT INCLUDE */ // DON´T TOUCH THIS CODE!!
#include "FAC_Code/mixes_functions/fac_mixes.h"
#include "FAC_Code/fac_settings.h"

/* CUSTOM INCLUDE */										// 8) of HOW TO MAKE A MIX
//#include "FAC_Code/fac_adc.h"
#include "FAC_Code/fac_imu.h"		// if get_status == HAL_ERROR NOT USE data!!

/* PRIVATE FUNCTIONS AND VARIABLES */
// marker only, it records which ID this file implements (see step 5): unused on purpose
static const uint8_t __attribute__((unused)) mix_id = FAC_MIX_SIMPLE_TANK_KEEP_DIRECTION;						// 3) of HOW TO MAKE A MIX		(only to know witch mix is this)

/* GRAPH STATE - file scope statics, zero initialized at boot, NOT touched by
 * FAC_mixes_init() / FAC_functions_init(), so they survive an apply and a disarm */
static uint32_t lastRun;
static fac_value_t edgeDetectPrev;
static fac_value_t sampleHoldHeld;
static int32_t pidAcc;
static fac_value_t pidPerr;
static uint32_t pidLast;
/* WHAT THIS MIX DO */											// 7) of HOW TO MAKE A MIX
/*
 * DESCRIPTION:
 * this mix makes the wheels to spin as a tank tracks, so implement a differential steering.
 *
 * WHY THE diff TERM EXISTS - DO NOT REPLACE THIS WITH A PLAIN SATURATED SUM !!
 * A transmitter gimbal moves inside a SQUARE gate, not a circular one: throttle and
 * steering can both be at their end of travel at the same time, in the corners of the
 * gate. A plain "left = throttle + steering" clipped at full scale would behave as if
 * the gate were circular, and every corner of the square would saturate to the same
 * value, losing the difference between them. The diff term redistributes the leftover
 * travel instead, so the corners of the gate stay distinguishable and the robot keeps
 * steering at full throttle.
 * The sums are deliberately NOT saturated: they are allowed to reach twice the full
 * scale and the / 2 below brings them back. The final FAC_math_clamp of the
 * boilerplate is what keeps the outputs in range - this graph must not add its own.
 *
 * INPUTs DESCRIPTION:
 * Uncomment the input you want to use and rename it
 * (ex: #define INPUT_THROTTLE 0, or #define INPUT_STEERING 1)
 * !! DONT'T CHANGE THE NUMBER ASIGNED !!
 * Use the INPUT_<NAME> to know the position of where to get the input value in the inputs array
 */
#define INPUT_THROTTLE 0
#define INPUT_STEERING 1
//#define INPUT_NOT_USED 2
//#define INPUT_NOT_USED 3
//#define INPUT_NOT_USED 4
//#define INPUT_NOT_USED 5

/* PARAMs DESCRIPTION:
 * Numbers (int16_t, -32768 to +32767) set from the FAC Tool, ex: the gains of a PID.
 * Uncomment the parameter you want to use and rename it (ex: #define PARAM_KP 0)
 * !! DONT'T CHANGE THE NUMBER ASIGNED !!
 * Read it with FAC_mixes_GET_param(PARAM_<NAME>): the value changes on the next apply settings
 */
#define PARAM_MILLI_KP 0
#define PARAM_MILLI_KI 1
#define PARAM_MILLI_KD 2
//#define PARAM_NOT_USED 3

/* OUTPUTs DESCRIPTION:
 * Uncomment the output you want to use and rename it
 * (ex: #define OUTPUT_MOTOR_LEFT 0, or #define OUTPUT_MOTOR_RIGHT 1)
 * !! DONT'T CHANGE THE NUMBER ASIGNED !!
 * Use the OUTPUT_<NAME> to know the position of where to put the output value in the outputs array
 *
 * !! REMEMBER:
 * 		- All outputs must be in a standard format, values from FAC_VALUE_MIN to FAC_VALUE_MAX (-1000 to +1000) !!
 * 			write here what each mix's output is (ex: 0) motor left, 1) motor right 2)not used ...)
 * 		- If an output is for a DC motor, a positive number is considered as forward movement, a negative number is considered as backwards movement of the DC motor
 * 		- If an output is for a Servo vomor/esc, -1000 is considered as 0 and +1000 is considered as 100% (in degrees for servos 0°-180°)
 */
#define OUTPUT_MOTOR_LEFT 0
#define OUTPUT_MOTOR_RIGHT 1
//#define OUTPUT_NOT_USED 2
//#define OUTPUT_NOT_USED 3
//#define OUTPUT_NOT_USED 4
//#define OUTPUT_NOT_USED 5
//#define OUTPUT_NOT_USED 6
//#define OUTPUT_NOT_USED 7
//#define OUTPUT_NOT_USED 8
//#define OUTPUT_NOT_USED 9

/*
 * @brief	Calculate the mix output values
 *
 */
void FAC_simple_tank_keep_direction_mix_update(void) {					// 4) of HOW TO MAKE A MIX
	// this code must be left as it is, DON'T TOUCH IT!
	fac_value_t outputs[MIXES_MAX_OUTPUTS_NUMBER];
	fac_value_t inputs[MIXES_MAX_INPUTS_NUMBER];
	FAC_mixes_update_mix_inputs();	// update the mix input in base of the settings and rx channels
	for (int i = 0; i < MIXES_MAX_OUTPUTS_NUMBER; i++) {
		outputs[i] = FAC_VALUE_ZERO;	// every output must start at zero, the unused ones stay there
	}
	for (int i = 0; i < MIXES_MAX_INPUTS_NUMBER; i++) {
		inputs[i] = FAC_mixes_GET_input(i);
	}
	/* INSERT YOUR CODE HERE -START- */						// 9) of HOW TO MAKE A MIX
	/* REMEMBER
	 * - inputs array contains all values of the channel requested for this mix
	 * - in the outputs array you have to write in the same order you written above all outputs for servos and motors
	 * 		outputs values must stay in this range [FAC_VALUE_MIN, FAC_VALUE_MAX], that is [-1000, +1000]
	 * - use the FAC_math_* primitives of fac_math.h, NEVER a float: this mcu has no fpu and every
	 * 		float operation is a library call of hundreds of clock cycles
	 */
	// write here the code of your mix
	uint32_t now = HAL_GetTick();
	/* the chain only runs in FAC_STATE_NORMAL, so a disarm freezes every static.
	 * More than 100 ms since the last pass means we were not running. */
	if ((uint32_t) (now - lastRun) > 100) {
		edgeDetectPrev = FAC_VALUE_ZERO;
		sampleHoldHeld = FAC_VALUE_ZERO;
		pidAcc = 0;
		pidPerr = FAC_VALUE_ZERO;
		pidLast = now;
	}
	lastRun = now;
	FAC_IMU_update();	/* the main loop does NOT do this for us */
	/* levelled data is only real once the arming calibration has passed:
	 * before it the frame is the identity, i.e. the board's own axes. */
	if (FAC_IMU_GET_status() != HAL_ERROR && FAC_IMU_GET_is_calibrated()) {
		fac_value_t inThrottle = inputs[INPUT_THROTTLE];
		fac_value_t inSteering = inputs[INPUT_STEERING];
		fac_value_t absThrottle = FAC_math_abs(inThrottle);
		/* the integrator is INSIDE this call: read it once per pass, into a local */
		int32_t heading = FAC_IMU_GET_heading();
		fac_value_t fromRange = FAC_math_from_range(heading, 0, 4095);
		uint8_t iMSteering = (FAC_math_abs(inSteering)) >= 10;
		uint8_t edgeDetect = (edgeDetectPrev > 0 && iMSteering <= 0);
		if (edgeDetect) {
			sampleHoldHeld = fromRange;
		}
		int32_t errorRaw = sampleHoldHeld - fromRange;
		uint8_t compare = errorRaw > 1000;
		uint8_t compare2 = errorRaw < -1000;
		int32_t select = compare ? (errorRaw - 2000) : (2000 + compare2);
		int32_t select2 = (compare2 || compare) ? select : errorRaw;
		uint8_t or = (FAC_IMU_GET_is_upside_down()) || iMSteering;
		int32_t paramMilliKp = FAC_mixes_GET_param(PARAM_MILLI_KP);
		int32_t paramMilliKd = FAC_mixes_GET_param(PARAM_MILLI_KD);
		int32_t paramMilliKi = FAC_mixes_GET_param(PARAM_MILLI_KI);
		int32_t clampTo = FAC_math_clamp_to(paramMilliKp * 100, 0, 50000);
		int32_t clampTo2 = FAC_math_clamp_to(paramMilliKi * 100, 0, 50000);
		int32_t clampTo3 = FAC_math_clamp_to(paramMilliKd * 100, 0, 50000);
		uint32_t dt = (uint32_t) (now - pidLast);
		pidLast = now;
		int32_t err = select2 - 0;
		int32_t iinc = FAC_math_mul_scaled(FAC_math_mul_scaled(err, clampTo2, 1000), dt, 1);
		pidAcc = FAC_math_clamp_to(pidAcc + iinc, -1000, 1000);
		int32_t pterm = FAC_math_mul_scaled(err, clampTo, 1000);
		int32_t derr = err - pidPerr;
		pidPerr = err;
		int32_t dterm = FAC_math_div_scaled(derr, dt, clampTo3);
		int32_t sum = pterm + pidAcc + dterm;
		int32_t pid = FAC_math_clamp_to(sum, -1000, 1000);
		int32_t select3 = or ? inSteering : pid;
		fac_value_t absSteering = FAC_math_abs(select3);
		int32_t diff = absThrottle - absSteering;
		int32_t absDiff = diff < 0 ? -diff : diff;
		int32_t leftSum = inThrottle + select3;
		int32_t rightSum = inThrottle - select3;
		uint8_t leftIsNeg = leftSum < 0;
		uint8_t rightIsNeg = rightSum < 0;
		int32_t leftMinus = leftSum - absDiff;
		int32_t leftPlus = leftSum + absDiff;
		int32_t rightMinus = rightSum - absDiff;
		int32_t rightPlus = rightSum + absDiff;
		int32_t leftAdjusted = leftIsNeg ? leftMinus : leftPlus;
		int32_t rightAdjusted = rightIsNeg ? rightMinus : rightPlus;
		int32_t left = leftAdjusted / 2;
		int32_t right = rightAdjusted / 2;
		outputs[OUTPUT_MOTOR_LEFT] = left;
		outputs[OUTPUT_MOTOR_RIGHT] = right;
		edgeDetectPrev = iMSteering;
	} else {
		/* IMU fallback: every output stays at FAC_VALUE_ZERO, which is what the
		 * boilerplate already zero-initialized them to. Nothing to do. */
	}
	/* INSERT YOUR CODE HERE -END- */
	// keep outputs in range
	for (int i = 0; i < MIXES_MAX_OUTPUTS_NUMBER; i++) {
		outputs[i] = FAC_math_clamp(outputs[i]);
	}
	// update outputs values on mixes struct
	FAC_mixes_update_mix_outputs(outputs);
}


/* FAC_MIX_EDITOR_PROJECT v3 - re-import this file to keep editing
FACMIX_JSON_BEGIN
{"schemaVersion":3,"mode":"mix","title":"simple_tank_keep_direction","author":"Floppy Lab","description":"this mix makes the wheels to spin as a tank tracks, so implement a differential steering.\n\nWHY THE diff TERM EXISTS - DO NOT REPLACE THIS WITH A PLAIN SATURATED SUM !!\nA transmitter gimbal moves inside a SQUARE gate, not a circular one: throttle and\nsteering can both be at their end of travel at the same time, in the corners of the\ngate. A plain \"left = throttle + steering\" clipped at full scale would behave as if\nthe gate were circular, and every corner of the square would saturate to the same\nvalue, losing the difference between them. The diff term redistributes the leftover\ntravel instead, so the corners of the gate stay distinguishable and the robot keeps\nsteering at full throttle.\nThe sums are deliberately NOT saturated: they are allowed to reach twice the full\nscale and the / 2 below brings them back. The final FAC_math_clamp of the\nboilerplate is what keeps the outputs in range - this graph must not add its own.","instances":1,"imuFallback":"zero","embedJson":true,"nodes":[{"id":"p1","t":"mix_input","x":-25,"y":-50,"p":{"idx":0,"name":"THROTTLE"},"name":"in throttle","note":"","bind":"auto","grp":""},{"id":"p2","t":"mix_input","x":-3150,"y":0,"p":{"idx":1,"name":"STEERING"},"name":"","note":"","bind":"auto","grp":""},{"id":"p3","t":"abs","x":200,"y":-75,"p":{},"name":"abs throttle","note":"","bind":"auto","grp":""},{"id":"p4","t":"abs","x":200,"y":0,"p":{},"name":"abs steering","note":"","bind":"auto","grp":""},{"id":"p5","t":"raw_sub","x":400,"y":-25,"p":{},"name":"diff","note":"","bind":"auto","grp":""},{"id":"p6","t":"raw_abs","x":575,"y":-25,"p":{},"name":"abs diff","note":"","bind":"auto","grp":""},{"id":"p7","t":"raw_add","x":200,"y":-175,"p":{},"name":"left sum","note":"","bind":"auto","grp":""},{"id":"p8","t":"raw_sub","x":200,"y":100,"p":{},"name":"right sum","note":"","bind":"auto","grp":""},{"id":"p9","t":"const","x":0,"y":-125,"p":{"v":0,"kind":"RAW"},"name":"","note":"","bind":"auto","grp":""},{"id":"p10","t":"compare","x":400,"y":-150,"p":{"op":"<"},"name":"left is neg","note":"","bind":"auto","grp":""},{"id":"p11","t":"compare","x":400,"y":50,"p":{"op":"<"},"name":"right is neg","note":"","bind":"auto","grp":""},{"id":"p12","t":"raw_sub","x":775,"y":-175,"p":{},"name":"left minus","note":"","bind":"auto","grp":""},{"id":"p13","t":"raw_add","x":775,"y":-75,"p":{},"name":"left plus","note":"","bind":"auto","grp":""},{"id":"p14","t":"raw_sub","x":775,"y":0,"p":{},"name":"right minus","note":"","bind":"auto","grp":""},{"id":"p15","t":"raw_add","x":775,"y":100,"p":{},"name":"right plus","note":"","bind":"auto","grp":""},{"id":"p16","t":"select","x":950,"y":-100,"p":{},"name":"left adjusted","note":"","bind":"auto","grp":""},{"id":"p17","t":"select","x":950,"y":0,"p":{},"name":"right adjusted","note":"","bind":"auto","grp":""},{"id":"p18","t":"raw_div_lit","x":1150,"y":-75,"p":{"k":2},"name":"left","note":"","bind":"auto","grp":""},{"id":"p19","t":"raw_div_lit","x":1150,"y":0,"p":{"k":2},"name":"right","note":"","bind":"auto","grp":""},{"id":"p20","t":"mix_output","x":1325,"y":-75,"p":{"idx":0,"name":"MOTOR_LEFT"},"name":"","note":"","bind":"auto","grp":""},{"id":"p21","t":"mix_output","x":1325,"y":0,"p":{"idx":1,"name":"MOTOR_RIGHT"},"name":"","note":"","bind":"auto","grp":""},{"id":"n22","t":"pid","x":-825,"y":450,"p":{"kp":25000,"ki":5,"kd":500,"ilo":-1000,"ihi":1000,"olo":-1000,"ohi":1000,"onResume":"reset"},"name":"","note":"","bind":"auto","grp":""},{"id":"n24","t":"imu_heading","x":-2750,"y":550,"p":{},"name":"","note":"","bind":"auto","grp":"real direction"},{"id":"n26","t":"raw_sub","x":-2250,"y":500,"p":{},"name":"error raw","note":"","bind":"auto","grp":"error"},{"id":"n27","t":"compare","x":-1875,"y":325,"p":{"op":">"},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n28","t":"const","x":-2050,"y":300,"p":{"v":1000,"kind":"RAW"},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n29","t":"select","x":-1550,"y":525,"p":{},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n30","t":"raw_sub","x":-1825,"y":500,"p":{},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n31","t":"const","x":-2000,"y":550,"p":{"v":2000,"kind":"RAW"},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n32","t":"compare","x":-1975,"y":675,"p":{"op":"<"},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n33","t":"const","x":-2150,"y":700,"p":{"v":-1000,"kind":"RAW"},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n36","t":"raw_add","x":-1800,"y":675,"p":{},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n37","t":"const","x":-1275,"y":400,"p":{"v":0,"kind":"RAW"},"name":"","note":"","bind":"auto","grp":""},{"id":"n38","t":"from_range","x":-2550,"y":550,"p":{"imin":0,"imax":4095},"name":"","note":"","bind":"auto","grp":""},{"id":"n39","t":"select","x":-1275,"y":525,"p":{},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n40","t":"or","x":-1550,"y":375,"p":{},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n69","t":"select","x":-325,"y":250,"p":{},"name":"","note":"","bind":"auto","grp":""},{"id":"n71","t":"abs","x":-2950,"y":-150,"p":{},"name":"","note":"","bind":"auto","grp":""},{"id":"n72","t":"compare","x":-2725,"y":-100,"p":{"op":">="},"name":"I'm steering","note":"","bind":"auto","grp":""},{"id":"n73","t":"const","x":-2950,"y":-75,"p":{"v":10,"kind":"RAW"},"name":"","note":"","bind":"auto","grp":"error"},{"id":"n75","t":"sample_hold","x":-2450,"y":375,"p":{"onResume":"reset"},"name":"","note":"","bind":"auto","grp":""},{"id":"n76","t":"edge","x":-2675,"y":375,"p":{"th":0,"mode":"falling","onResume":"reset"},"name":"","note":"","bind":"auto","grp":""},{"id":"n84","t":"imu_upside_down","x":-725,"y":100,"p":{},"name":"","note":"","bind":"auto","grp":""},{"id":"n85","t":"or","x":-525,"y":175,"p":{},"name":"","note":"","bind":"auto","grp":""},{"id":"n88","t":"mix_param","x":-1675,"y":750,"p":{"idx":0,"name":"milli_KP"},"name":"","note":"","bind":"auto","grp":""},{"id":"n89","t":"mix_param","x":-1675,"y":900,"p":{"idx":2,"name":"milli_KD"},"name":"","note":"","bind":"auto","grp":""},{"id":"n90","t":"mix_param","x":-1675,"y":825,"p":{"idx":1,"name":"milli_KI"},"name":"","note":"","bind":"auto","grp":""},{"id":"n97","t":"clamp_to","x":-1350,"y":750,"p":{"lo":0,"hi":50000},"name":"","note":"","bind":"auto","grp":""},{"id":"n99","t":"raw_mul_lit","x":-1500,"y":750,"p":{"k":100},"name":"","note":"","bind":"auto","grp":""},{"id":"n101","t":"clamp_to","x":-1350,"y":825,"p":{"lo":0,"hi":50000},"name":"","note":"","bind":"auto","grp":""},{"id":"n102","t":"raw_mul_lit","x":-1500,"y":825,"p":{"k":100},"name":"","note":"","bind":"auto","grp":""},{"id":"n103","t":"clamp_to","x":-1350,"y":900,"p":{"lo":0,"hi":50000},"name":"","note":"","bind":"auto","grp":""},{"id":"n104","t":"raw_mul_lit","x":-1500,"y":900,"p":{"k":100},"name":"","note":"","bind":"auto","grp":""}],"wires":[{"from":"p1","to":"p3","port":0},{"from":"p3","to":"p5","port":0},{"from":"p4","to":"p5","port":1},{"from":"p5","to":"p6","port":0},{"from":"p1","to":"p7","port":0},{"from":"p1","to":"p8","port":0},{"from":"p7","to":"p10","port":0},{"from":"p9","to":"p10","port":1},{"from":"p8","to":"p11","port":0},{"from":"p9","to":"p11","port":1},{"from":"p7","to":"p12","port":0},{"from":"p6","to":"p12","port":1},{"from":"p7","to":"p13","port":0},{"from":"p6","to":"p13","port":1},{"from":"p8","to":"p14","port":0},{"from":"p6","to":"p14","port":1},{"from":"p8","to":"p15","port":0},{"from":"p6","to":"p15","port":1},{"from":"p10","to":"p16","port":0},{"from":"p12","to":"p16","port":1},{"from":"p13","to":"p16","port":2},{"from":"p11","to":"p17","port":0},{"from":"p14","to":"p17","port":1},{"from":"p15","to":"p17","port":2},{"from":"p16","to":"p18","port":0},{"from":"p17","to":"p19","port":0},{"from":"p18","to":"p20","port":0},{"from":"p19","to":"p21","port":0},{"from":"n26","to":"n27","port":0},{"from":"n28","to":"n27","port":1},{"from":"n27","to":"n29","port":0},{"from":"n26","to":"n30","port":0},{"from":"n31","to":"n30","port":1},{"from":"n30","to":"n29","port":1},{"from":"n33","to":"n32","port":1},{"from":"n26","to":"n32","port":0},{"from":"n31","to":"n36","port":0},{"from":"n32","to":"n36","port":1},{"from":"n36","to":"n29","port":2},{"from":"n24","to":"n38","port":0},{"from":"n38","to":"n26","port":1},{"from":"n29","to":"n39","port":1},{"from":"n26","to":"n39","port":2},{"from":"n32","to":"n40","port":0},{"from":"n27","to":"n40","port":1},{"from":"n40","to":"n39","port":0},{"from":"n69","to":"p8","port":1},{"from":"n69","to":"p4","port":0},{"from":"n69","to":"p7","port":1},{"from":"n22","to":"n69","port":2},{"from":"p2","to":"n69","port":1},{"from":"p2","to":"n71","port":0},{"from":"n71","to":"n72","port":0},{"from":"n73","to":"n72","port":1},{"from":"n75","to":"n26","port":0},{"from":"n72","to":"n76","port":0},{"from":"n76","to":"n75","port":1},{"from":"n38","to":"n75","port":0},{"from":"n39","to":"n22","port":0},{"from":"n37","to":"n22","port":1},{"from":"n84","to":"n85","port":0},{"from":"n72","to":"n85","port":1},{"from":"n85","to":"n69","port":0},{"from":"n97","to":"n22","port":3},{"from":"n88","to":"n99","port":0},{"from":"n99","to":"n97","port":0},{"from":"n102","to":"n101","port":0},{"from":"n104","to":"n103","port":0},{"from":"n90","to":"n102","port":0},{"from":"n89","to":"n104","port":0},{"from":"n101","to":"n22","port":4},{"from":"n103","to":"n22","port":5}],"collapsed":{"real direction":false,"target direction":false,"error":false},"view":{"x":2211.701824253455,"y":-552.3207799184003,"z":1.2383649997217163},"reg":{"mixCount":2,"fnCount":10},"sim":{"deadzonePerc":2,"chAssign":[2,1,0,0,0,0],"reverse":[0,1,0,0,0,0],"params":[314,0,0,0],"fnChannels":[3,0,0,0,0,0,0,0,0],"stage0":[500,500,0,500,500,500,500,500],"mapper":{"M1":100,"M2":101,"M3":0,"S1":200,"S2":0},"rcHz":50,"speed":1,"disarmMs":3000,"gyro":[0,0,-14032],"accel":[0,0,2049],"gen":"const","genAmp":8000,"genHz":1,"imuFail":0,"batteryMv":8000,"cells":2,"adc":[2048,2048,1500],"rxConnected":1,"robot":{"maxSpeedCms":120,"shape":[[10,8],[10,-8],[-10,-8],[-10,8]],"wheels":[{"x":0,"y":7,"deg":0,"grip":1,"link":"M1","steer":"","steerDeg":30,"r":3.5},{"x":0,"y":-7,"deg":0,"grip":1,"link":"M2","steer":"","steerDeg":30,"r":3.5}],"devices":[{"x":8,"y":0,"link":"S1","label":""}],"imu":{"x":0,"y":0},"imuSource":"robot","mountRollDeg":0,"mountPitchDeg":0,"mountYawDeg":0,"tiltRollDeg":0,"tiltPitchDeg":0,"gyroNoise":0,"accelNoise":0,"gyroBias":0,"accelBias":0,"viewCm":120,"gridMm":10,"gridSnap":1,"follow":1,"trail":1}},"pad":{"index":0,"bindings":[{"src":"axis","idx":2,"invert":0,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"axis","idx":3,"invert":1,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"axis","idx":1,"invert":1,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"axis","idx":0,"invert":0,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"button","idx":6,"invert":0,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"button","idx":7,"invert":0,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"none","idx":0,"invert":0,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0},{"src":"none","idx":0,"invert":0,"center":0,"min":-1,"max":1,"dz":0.05,"btn":"momentary","tog":0,"pos":0,"prev":0}]}}
FACMIX_JSON_END
*/
