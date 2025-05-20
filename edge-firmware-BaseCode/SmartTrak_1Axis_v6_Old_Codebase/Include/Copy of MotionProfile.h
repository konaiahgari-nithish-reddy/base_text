// *************************************************************************************************
//										M o t i o n P r o f i l e . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Profile Table
//
//		001	19 Mar 13 <sek> created from gsf Code Base and initial spreadsheet SmartTrak Solar Tracker Motion Calculations A01.XLS
//		002 24 Mar 13 <sek> added 3rd column; simulator timing values from spreadsheet SmartTrak Solar Tracker Motion Calculations A02.XLS
//		003 25 Mar 13 <sek> MOTION_PROFILE_TABLE_WIDTH
//		004 26 Mar 13 <sek> moved motion description macros here from MotionLimit.h
//		005 01 Apr 13 <sek> limited to needed motions, rearranged
//		006 10 Apr 13 <sek> additional MotionProfile table for Slew 236:1 gearbox, SLEW_575_TO_1, SLEW_236_TO_1
//		007 11 Apr 13 <sek> moved MTN_SENSOR_SPEED_TOLERANCE here from MotionSensor.h; dependent on Speed values
//		008 24 Apr 13 <sek> removed some unused macros
//		009	29 Apr 13 <sek> changed STALL_RECOVERY_MIN_INDEX from 27 to 7 due to MC33926 shutdowns when starting FORWARD recovery
//		010 24 May 13 <sek> RUN_MINIMUM_MOVE_TICKS
//		011 10 Jun 13 <sek> changed maximum speed for slew from 50% tp 75%
//		012 30 Jul 13 <sek> array of MotionProfileSpeedAndPWM[][] to allow separate tables for AZ, EL axes
//		013 09 Aug 13 <sek> 58:1 Elevation MotionProfileSpeedAndPWM[][] table, conditional operator in macros for axis selection
//		014 13 Aug 13 <sek> removed unneeded motor parameter from MIN_SPEED_INDEX(motor), RUN_MOTION_PROFILE_INDEX_MIN(), SLEW_MOTION_PROFILE_INDEX_MIN()
//		015 15 Aug 13 <sek> corrected GLOBAL definition MotionProfileSpeedAndPWM[NUM_MOTORS][(MOTION_PROFILE_TABLE_LEN * 2) + 2];
//		016 23 Aug 13 <sek> new table for elevation linear actuator, MOTION_PROFILE_TBL_TYPE
//		017 02 Sep 13 <sek> MOTION_PROFILE_SPEED_TYPE, MOTION_PROFILE_SPEED_ERR_TYPE
//
//		AUTHOR:	    Steve Kranish	skranish@verizon.net
//					gsf Engineering	978-927-7189
//					Beverly, MA 01915
//
//		copyright (c) 2013 gsf Engineering (Beverly, MA USA) for SmartTrak Solar Systems Pvt, Hyderabad, AP, India
//
// *************************************************************************************************

// Motion Profile table limits, see below


// *************************************
//			Table Length
// *************************************
#define	AZ_MOTION_PROFILE_TABLE_LEN				(124 + 1)							// manually taken from MotionProfileSpeedAndPWM[MOTOR_AZIMUTH][] table below, 124 + 1

#if defined(USE_ELEVATION_SLEW_DRIVE)
	#define	EL_MOTION_PROFILE_TABLE_LEN			(139 + 1)							// manually taken from MotionProfileSpeedAndPWM[MOTOR_ELEVATION][] table below, 139 + 1
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_MOTION_PROFILE_TABLE_LEN			(62 + 1)							// manually taken from MotionProfileSpeedAndPWM[MOTOR_ELEVATION][] table below, 62 + 1
#else
	#error Elevation Drive Type must be specified
#endif


#if defined(USE_ELEVATION_SLEW_DRIVE)
	// MOTION_PROFILE_TABLE_LEN is the longer of the two tables; the shorter table will have uninitialized data that is not used
	#define	MOTION_PROFILE_TABLE_LEN			EL_MOTION_PROFILE_TABLE_LEN		// longer of two tables
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	MOTION_PROFILE_TABLE_LEN			AZ_MOTION_PROFILE_TABLE_LEN		// longer of two tables
#endif


// These macros implement axis selection by the C Conditional operator
//	(condition ? value for true : value for not true)

// *************************************
//			Speed Indicies
// *************************************
// important values for x.xxx degrees/sec/sec acceleration and y.yy degrees/s maximum speed
// total table length is xx_MOTION_PROFILE_TABLE_LEN, from 28% to 100% duty cycle (note that PWM values account for error introduced by MC33926 finite PWM rise and fall times)
#define	MIN_SPEED_INDEX							1								// manually taken from MotionProfileSpeedAndPWM[] table below
#define	MIN_PWM_INDEX							MIN_SPEED_INDEX

#define	MAX_SPEED_INDEX(x)						((x == MOTOR_AZIMUTH) ? (AZ_MOTION_PROFILE_TABLE_LEN - 1) : (EL_MOTION_PROFILE_TABLE_LEN - 1))
#define	MAX_PWM_INDEX(x)						MAX_SPEED_INDEX(x)

//#define	RUN_MAX_SPEED_INDEX(x)					MAX_SPEED_INDEX(x)

#define	AZ_SLEW_MAX_SPEED_INDEX					68								// table index for 0.75 maximum speed, see table below

#if defined(USE_ELEVATION_SLEW_DRIVE)
	#define	EL_SLEW_MAX_SPEED_INDEX				76								// table index for 0.75 maximum speed, see table below
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_SLEW_MAX_SPEED_INDEX				34								// table index for 0.75 maximum speed, see table below
#endif
#define	SLEW_MAX_SPEED_INDEX(x)					((x == MOTOR_AZIMUTH) ? AZ_SLEW_MAX_SPEED_INDEX : EL_SLEW_MAX_SPEED_INDEX)

// *************************************
//			Stall Indicies
// *************************************

// index values used during stall recovery; limited to prevent breaking something!
// ==> these values are estimates, and are taken from the MotionProfile tables BELOW
#define	AZ_STALL_RECOVERY_MIN_INDEX				7			// one third speed, PWM 36%
#define	AZ_STALL_RECOVERY_MAX_INDEX				67			// 77%
#define	AZ_STALL_RECOVERY_INCREMENT				4			// 15 steps, 7 to 67


#if defined(USE_ELEVATION_SLEW_DRIVE)
	#define	EL_STALL_RECOVERY_MIN_INDEX			7			// one third speed, PWM 36%
	#define	EL_STALL_RECOVERY_MAX_INDEX			77			// 75% speed
	#define	EL_STALL_RECOVERY_INCREMENT			5			// 14 steps, 7 to 77
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_STALL_RECOVERY_MIN_INDEX			5			// one third speed, PWM 36%
	#define	EL_STALL_RECOVERY_MAX_INDEX			35			// 75% speed
	#define	EL_STALL_RECOVERY_INCREMENT			3			// 10 steps, 5 to 35
#endif
#define	SLEW_MAX_SPEED_INDEX(x)					((x == MOTOR_AZIMUTH) ? AZ_SLEW_MAX_SPEED_INDEX : EL_SLEW_MAX_SPEED_INDEX)



#define	STALL_RECOVERY_MIN_INDEX(x)				((x == MOTOR_AZIMUTH) ? AZ_STALL_RECOVERY_MIN_INDEX : EL_STALL_RECOVERY_MIN_INDEX)
#define	STALL_RECOVERY_MAX_INDEX(x)				((x == MOTOR_AZIMUTH) ? AZ_STALL_RECOVERY_MAX_INDEX : EL_STALL_RECOVERY_MAX_INDEX)
#define	STALL_RECOVERY_INCREMENT(x)				((x == MOTOR_AZIMUTH) ? AZ_STALL_RECOVERY_INCREMENT : EL_STALL_RECOVERY_INCREMENT)

// *********************************************************
//		Move Definitions, Based on Motion Profile
// *********************************************************

// *************************************
//			RUN type move
// *************************************
// A 'RUN' type move accelerates to maximum possible speed.
// This ratio defines the shape of the motion profile for RUN. MAY NEED TO BE AXIS SPECIFIC
#define	RUN_ACCEL_TO_DECEL_RATIO				4							// MSI Ticks acceleration to deceleration ratio

#define	AZ_RUN_MINIMUM_MOVE_TICKS				5							// minimum move size
#define	EL_RUN_MINIMUM_MOVE_TICKS				5							// minimum move size, MAY NEED TO BE CHANGED; untested
#define	RUN_MINIMUM_MOVE_TICKS(x)				((x == MOTOR_AZIMUTH) ? AZ_RUN_MINIMUM_MOVE_TICKS : EL_RUN_MINIMUM_MOVE_TICKS)

// Deceleration rate is 4 x the rate of acceleration (#defined above, may need to be adjusted)
#define	MSI_RUN_ACCELERATION_COUNT(x)			(MAX_SPEED_INDEX(x) - MIN_SPEED_INDEX)								// Motion Sensor ticks for complete acceleration ramp (increment by 1) See Motion Math page of Visio documentation
#define	MSI_RUN_DECELERATION_COUNT(x)			(MSI_RUN_ACCELERATION_COUNT(x)/RUN_ACCEL_TO_DECEL_RATIO)			// Motion Sensor ticks for complete deceleration ramp, may include Minimum PWM ticks

// MOVE refers to both SHORT and LONG moves, which use identical motion profiles, with only the run being different lengths
#define	RUN_MOTION_PROFILE_INDEX_MIN			MIN_SPEED_INDEX
#define	RUN_MOTION_PROFILE_INDEX_MAX(x)			(MAX_SPEED_INDEX(x) - 1)
#define	RUN_MOTION_PROFILE_INDEX_INCREMENT		1
#define	RUN_MOTION_PROFILE_INDEX_DECREMENT		RUN_ACCEL_TO_DECEL_RATIO

//#if (RUN_MOTION_PROFILE_INDEX_DECREMENT < 1)
//	#error Motion Profile Math Error
//#endif

// *************************************
//			SLEW type move
// *************************************
// slew, used for SLEW_XXX_TO_END movement types
// maximum length move, with end-of-travel detection
// note that this is a low acceleration, low speed move, with NO deceleration phase because motion is terminated by the Limit Switch
#define	MSI_SLEW_ACCELERATION_COUNT(x)			(SLEW_MAX_SPEED_INDEX(x)  - MIN_SPEED_INDEX)								// Motion Sensor ticks for complete acceleration ramp (increment by 1)
#define	MSI_SLEW_DECELERATION_COUNT				0																			// no deceleration phase; motion is stopped by limit switch
#define	MSI_SLEW_MAXIMUM_COUNT(x)				AZ_MSI_TICKS_ALLOWABLE														// Motion Sensor ticks for maximum possible move to end
#define	MSI_SLEW_CONSTANT_SP_COUNT(x)			(MSI_SLEW_MAXIMUM_COUNT(x) - (MSI_SLEW_ACCELERATION_COUNT(x) + MSI_SLEW_DECELERATION_COUNT))	// Motion Sensor ticks for constant speed run motion

#define	SLEW_MOTION_PROFILE_INDEX_MIN			MIN_SPEED_INDEX
#define	SLEW_MOTION_PROFILE_INDEX_MAX(x)		SLEW_MAX_SPEED_INDEX(x)
#define	SLEW_MOTION_PROFILE_INDEX_INCREMENT		1
#define	SLEW_MOTION_PROFILE_INDEX_DECREMENT		0																			// no deceleration phase


// *************************************
//		Stall Recovery type move
// *************************************
// stall recovery
// ==>> these values are untested, and may need to vary with gear ratios
// stall recovery is based on TIMER ticks, not Motion Sensor ticks, because... we may not be moving!
#define	MSI_STALL_RECOVERY_ACCELERATION_COUNT(x)	((STALL_RECOVERY_MAX_INDEX(x) - STALL_RECOVERY_MIN_INDEX(x)) / STALL_RECOVERY_INCREMENT(x))		// we need to see only TWO ticks to know we are in motion, but MORE to get unstuck!!
#define	MSI_STALL_RECOVERY_CONSTANT_SP_COUNT(x)		(MSI_STALL_RECOVERY_ACCELERATION_COUNT(x) * 4)												// in stall recovery, this means the number of 25mS ticks for which we apply Maximum Stall PWM
#define	MSI_STALL_RECOVERY_DECELERATION_COUNT(x)	(MSI_STALL_RECOVERY_ACCELERATION_COUNT(x) * 2)
//#define	MSI_STALL_RECOVERY_MAXIMUM_COUNT		(MSI_STALL_RECOVERY_ACCELERATION_COUNT + MSI_STALL_RECOVERY_DECELERATION_COUNT)

#define	STALL_RECOVERY_MOTION_PROFILE_INDEX_MIN(x)			STALL_RECOVERY_MIN_INDEX(x)
#define	STALL_RECOVERY_MOTION_PROFILE_INDEX_MAX(x)			STALL_RECOVERY_MAX_INDEX(x)
#define	STALL_RECOVERY_MOTION_PROFILE_INDEX_INCREMENT(x)	STALL_RECOVERY_INCREMENT(x)
#define	STALL_RECOVERY_MOTION_PROFILE_INDEX_DECREMENT(x)	(STALL_RECOVERY_INCREMENT(x) / 2)

#define	SOFT_STALL_RECOVERY_DELAY_COUNT			40			// 25mS ticks to delay between Soft Stall recovery substates


// stall recovery direction reversal motion boundaries
// outside these boundaries, stall recovery is ALWAYS towards zero, regardless of direction of motion that stalled
#define MSI_STALL_RECOVERY_REVERSE_LIMIT(x)		((x == MOTOR_AZIMUTH) ? AZ_ALLOWABLE_POS_REVERSE : EL_ALLOWABLE_POS_REVERSE)		// value accounts for staying away from the Limit Switches
#define MSI_STALL_RECOVERY_FORWARD_LIMIT(x)		((x == MOTOR_AZIMUTH) ? AZ_ALLOWABLE_POS_FORWARD : EL_ALLOWABLE_POS_FORWARD)

//#define	MTN_SENSOR_SPEED_MAX			xx				// likely fastest speed with 24V power supply

#define	MTN_SENSOR_SPEED_TOLERANCE			100				// difference between expected and counted speed for speed adjustment, ==>> perhaps this should be part of the spreadsheet?


// *********************************************************
//					Global Variables
// *********************************************************


#ifdef DEFINE_GLOBALS
	GLOBAL_INIT BYTE pgbMotionProfileIndexIncrement[NUM_MOTORS] = {0, 0};		// index step size during acceleration
	GLOBAL_INIT BYTE pgbMotionProfileIndexDecrement[NUM_MOTORS] = {0, 0};		// index step size during deceleration
	GLOBAL_INIT BYTE pgbMotionProfileIndexMin[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT BYTE pgbMotionProfileIndexMax[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT BYTE pgbMotionProfileSpeedIndex[NUM_MOTORS] = {0, 0};
#else
	GLOBAL BYTE pgbMotionProfileIndexIncrement[];
	GLOBAL BYTE pgbMotionProfileIndexDecrement[];
	GLOBAL BYTE pgbMotionProfileIndexMin[];
	GLOBAL BYTE pgbMotionProfileIndexMax[];
	GLOBAL BYTE pgbMotionProfileSpeedIndex[];
#endif


// *********************************************************
//			Motion Profile Table Definitions
// *********************************************************

#if defined(USE_ELEVATION_SLEW_DRIVE)
	typedef	UINT16	MOTION_PROFILE_TBL_TYPE;		// speed values all fit in unsigned 16 bit value
	typedef	UINT16	MOTION_PROFILE_SPEED_TYPE;		// speed values all fit in unsigned 16 bit value
	typedef	INT16S	MOTION_PROFILE_SPEED_ERR_TYPE;	// speed error values all fit in signed 16 bit value
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	typedef	UINT32	MOTION_PROFILE_TBL_TYPE;		// slowest speed values reqiure a 32 bit value
	typedef	UINT32	MOTION_PROFILE_SPEED_TYPE;		// speed values all fit in unsigned 32 bit value
	typedef	INT32S	MOTION_PROFILE_SPEED_ERR_TYPE;	// speed error values all fit in signed 32 bit value
#endif


// index offsets into MotionProfileSpeedAndPWM[] table (array) below
#define	MSI_SPEED_OFFSET				0
#define	PWM_VALUE_OFFSET				1

#ifdef USE_FEEDBACK_SIMULATOR
	#define	SIMULATOR_5MS_TICK_OFFSET		2
	#define	MOTION_PROFILE_TABLE_WIDTH		3		// MotionProfileSpeedAndPWM[][] is treated as an array, so this is the offset required for each step (row)
	#define	SIMULATOR_TCY_TICK_DIVIDER		781		// calculated value, converts Tcy ticks into 5ms Ticks
#else
	#define	MOTION_PROFILE_TABLE_WIDTH		2		// MotionProfileSpeedAndPWM[][] is treated as an array, so this is the offset required for each step (row)
#endif

// *********************************************************
//				Motion Profile Table
// *********************************************************

// This table contains the PWM setting in percent and the expected MSI Tick speed in Tcy clock cycles
// for EACH MSI tick. The array index (x2) is actually the MSI Tick count.

//         MSI Tick
// Time in     PWM Duty Cycle
// Tcy Ticks	in percent
// "Speed'     


#ifdef DEFINE_GLOBALS

	GLOBAL_INIT	ARRAY const MOTION_PROFILE_TBL_TYPE MotionProfileSpeedAndPWM[NUM_MOTORS][(MOTION_PROFILE_TABLE_LEN * 2) + 2] =
	{
		{
		#warning Azimuth Motor Gearbox 236:1 + Slew Gearbox 73:1
		// Ticks       %dc		  step
		// "Speed”,	  in percent
			0,			0,		//  0		NOTE: this row does NOT come from the spreadsheet
			14119,		28,		//  1
			13091,		30,		//  2
			12258,		31,		//  3
			11567,		32,		//  4
			10981,		34,		//  5
			10476,		35,		//  6
			10034,		36,		//  7
			9644,		37,		//  8
			9297,		38,		//  9
			8984,		39,		//  10
			8701,		40,		//  11
			8443,		41,		//  12
			8207,		42,		//  13
			7990,		43,		//  14
			7788,		44,		//  15
			7602,		45,		//  16		// 1 second
			7428,		46,		//  17
			7266,		47,		//  18
			7114,		48,		//  19
			6971,		49,		//  20
			6836,		49,		//  21
			6709,		50,		//  22
			6588,		51,		//  23
			6474,		52,		//  24
			6366,		52,		//  25
			6263,		53,		//  26
			6164,		54,		//  27		// half speed, limit for slew, starting point for stall recovery
			6071,		55,		//  28
			5981,		55,		//  29
			5895,		56,		//  30
			5813,		57,		//  31
			5734,		57,		//  32
			5659,		58,		//  33
			5586,		59,		//  34
			5516,		59,		//  35
			5448,		60,		//  36
			5383,		60,		//  37
			5320,		61,		//  38
			5260,		62,		//  39
			5201,		62,		//  40
			5145,		63,		//  41
			5090,		63,		//  42
			5036,		64,		//  43
			4985,		65,		//  44
			4935,		65,		//  45
			4886,		66,		//  46
			4839,		66,		//  47
			4794,		67,		//  48
			4749,		67,		//  49
			4706,		68,		//  50
			4664,		69,		//  51
			4623,		69,		//  52
			4583,		70,		//  53
			4544,		70,		//  54
			4506,		71,		//  55
			4469,		71,		//  56
			4433,		72,		//  57
			4398,		72,		//  58
			4363,		73,		//  59
			4329,		73,		//  60
			4297,		74,		//  61
			4264,		74,		//  62
			4233,		75,		//  63
			4202,		75,		//  64
			4172,		76,		//  65
			4143,		76,		//  66
			4114,		77,		//  67
			4086,		77,		//  68		75% of maximum speed, for alternate slew
			4058,		78,		//  69
			4031,		78,		//  70
			4004,		79,		//  71
			3978,		79,		//  72
			3953,		79,		//  73
			3928,		80,		//  74
			3903,		80,		//  75
			3879,		81,		//  76
			3855,		81,		//  77
			3832,		82,		//  78
			3809,		82,		//  79
			3787,		83,		//  80
			3765,		83,		//  81
			3743,		83,		//  82
			3722,		84,		//  83
			3701,		84,		//  84
			3680,		85,		//  85
			3660,		85,		//  86
			3640,		86,		//  87
			3621,		86,		//  88
			3601,		86,		//  89
			3582,		87,		//  90
			3564,		87,		//  91
			3545,		88,		//  92
			3527,		88,		//  93
			3509,		88,		//  94
			3492,		89,		//  95
			3474,		89,		//  96
			3457,		90,		//  97
			3441,		90,		//  98
			3424,		90,		//  99
			3408,		91,		//  100
			3392,		91,		//  101
			3376,		92,		//  102
			3360,		92,		//  103
			3345,		92,		//  104
			3329,		93,		//  105
			3314,		93,		//  106
			3300,		94,		//  107
			3285,		94,		//  108
			3271,		94,		//  109
			3256,		95,		//  110
			3242,		95,		//  111
			3228,		95,		//  112
			3215,		96,		//  113
			3201,		96,		//  114
			3188,		97,		//  115
			3175,		97,		//  116
			3162,		97,		//  117
			3149,		98,		//  118
			3136,		98,		//  119
			3123,		98,		//  120
			3111,		99,		//  121
			3099,		99,		//  122
			3087,		99,		//  123
			3075,		100,	//  124
			3063,		100		//  125
		},

#ifdef USE_ELEVATION_SLEW_DRIVE
		{
		#warning Elevation Slew Gearbox 58:1
		#warning using 2 ticks per motor revolution
		// Ticks    %dc			step
		// "Speed”,	in percent
			0,			0,		//  0		NOTE: this row does NOT come from the spreadsheet
			11268,		28,		//	1
			10533,		29,		//	2
			9925,		31,		//	3
			9412,		32,		//	4
			8970,		33,		//	5
			8586,		34,		//	6
			8247,		35,		//	7
			7945,		36,		//	8
			7675,		37,		//	9
			7430,		38,		//	10
			7207,		39,		//	11
			7003,		40,		//	12
			6815,		41,		//	13
			6642,		42,		//	14
			6481,		43,		//	15
			6332,		44,		//	16
			6192,		44,		//	17
			6061,		45,		//	18
			5939,		46,		//	19
			5823,		47,		//	20
			5714,		47,		//	21	1 Second
			5610,		48,		//	22
			5513,		49,		//	23
			5420,		50,		//	24
			5331,		50,		//	25
			5247,		51,		//	26
			5167,		52,		//	27
			5090,		52,		//	28
			5017,		53,		//	29
			4946,		54,		//	30	Half Speed
			4879,		54,		//	31
			4814,		55,		//	32
			4752,		55,		//	33
			4692,		56,		//	34
			4634,		57,		//	35
			4579,		57,		//	36
			4525,		58,		//	37
			4473,		58,		//	38
			4423,		59,		//	39
			4375,		60,		//	40
			4328,		60,		//	41
			4283,		61,		//	42
			4239,		61,		//	43
			4196,		62,		//	44
			4154,		62,		//	45
			4114,		63,		//	46
			4075,		63,		//	47
			4037,		64,		//	48
			4000,		64,		//	49
			3964,		65,		//	50
			3929,		65,		//	51
			3895,		66,		//	52
			3862,		66,		//	53
			3830,		67,		//	54
			3798,		67,		//	55
			3767,		68,		//	56
			3737,		68,		//	57
			3708,		69,		//	58
			3679,		69,		//	59
			3651,		70,		//	60
			3624,		70,		//	61
			3597,		71,		//	62
			3571,		71,		//	63
			3545,		72,		//	64
			3520,		72,		//	65
			3496,		72,		//	66
			3472,		73,		//	67
			3448,		73,		//	68
			3425,		74,		//	69
			3402,		74,		//	70
			3380,		75,		//	71
			3358,		75,		//	72
			3337,		76,		//	73
			3316,		76,		//	74
			3296,		76,		//	75
			3275,		77,		//	76		 75% of full speed
			3256,		77,		//	77
			3236,		78,		//	78
			3217,		78,		//	79
			3198,		78,		//	80
			3180,		79,		//	81
			3162,		79,		//	82
			3144,		80,		//	83
			3126,		80,		//	84
			3109,		80,		//	85
			3092,		81,		//	86
			3075,		81,		//	87
			3059,		82,		//	88
			3043,		82,		//	89
			3027,		82,		//	90
			3011,		83,		//	91
			2996,		83,		//	92
			2981,		84,		//	93
			2966,		84,		//	94
			2951,		84,		//	95
			2937,		85,		//	96
			2922,		85,		//	97
			2908,		85,		//	98
			2894,		86,		//	99
			2881,		86,		//	100
			2867,		87,		//	101
			2854,		87,		//	102
			2841,		87,		//	103
			2828,		88,		//	104
			2815,		88,		//	105
			2802,		88,		//	106
			2790,		89,		//	107
			2778,		89,		//	108
			2765,		89,		//	109
			2754,		90,		//	110
			2742,		90,		//	111
			2730,		91,		//	112
			2719,		91,		//	113
			2707,		91,		//	114
			2696,		92,		//	115
			2685,		92,		//	116
			2674,		92,		//	117
			2663,		93,		//	118
			2652,		93,		//	119
			2642,		93,		//	120
			2631,		94,		//	121
			2621,		94,		//	122
			2611,		94,		//	123
			2601,		95,		//	124
			2591,		95,		//	125
			2581,		95,		//	126
			2571,		96,		//	127
			2562,		96,		//	128
			2552,		96,		//	129
			2543,		97,		//	130
			2534,		97,		//	131
			2524,		97,		//	132
			2515,		98,		//	133
			2506,		98,		//	134
			2497,		98,		//	135
			2489,		99,		//	136
			2480,		99,		//	137
			2471,		99,		//	138
			2463,		100,	//	139			100% PWM value, no need for more values. Note STEP number
			2454,		100,	//	140			one extra value so we do not run off the table
		}
	};
#endif	// USE_ELEVATION_SLEW_DRIVE

#ifdef USE_ELEVATION_LINEAR_DRIVE
		{

		#warning Elevation Linear Actuator 58:1
		#warning using 64 ticks per Linear Inch
		// Ticks    %dc			step
		// "Speed”,	in percent
			0,			0,		//  0		NOTE: this row does NOT come from the spreadsheet
			102437,		28,		//	1	Starting PWM value
			88880,		31,		//	2
			79587,		34,		//	3
			72707,		36,		//	4
			67350,		38,		//	5
			63026,		41,		//	6
			59440,		42,		//	7
			56404,		44,		//	8
			53791,		46,		//	9
			51510,		48,		//	10
			49496,		49,		//	11
			47701,		51,		//	12
			46089,		52,		//	13
			44630,		54,		//	14	Half Speed
			43301,		55,		//	15
			42084,		57,		//	16
			40964,		58,		//	17
			39929,		59,		//	18
			38969,		61,		//	19
			38075,		62,		//	20
			37240,		63,		//	21
			36457,		64,		//	22
			35722,		65,		//	23
			35030,		66,		//	24
			34376,		68,		//	25
			33758,		69,		//	26
			33171,		70,		//	27
			32615,		71,		//	28
			32085,		72,		//	29
			31581,		73,		//	30
			31099,		74,		//	31
			30639,		75,		//	32
			30199,		76,		//	33	 1 Second
			29777,		77,		//	34	 75% of full speed
			29372,		78,		//	35
			28984,		79,		//	36
			28610,		80,		//	37
			28251,		81,		//	38
			27904,		81,		//	39
			27570,		82,		//	40
			27248,		83,		//	41
			26937,		84,		//	42
			26637,		85,		//	43
			26346,		86,		//	44
			26064,		87,		//	45
			25792,		87,		//	46
			25527,		88,		//	47
			25271,		89,		//	48
			25022,		90,		//	49
			24781,		91,		//	50
			24546,		91,		//	51
			24318,		92,		//	52
			24096,		93,		//	53
			23880,		94,		//	54
			23670,		95,		//	55
			23465,		95,		//	56
			23265,		96,		//	57
			23071,		97,		//	58
			22881,		98,		//	59
			22696,		98,		//	60
			22515,		99,		//	61
			22339,		100,	//	62		100% PWM value, no need for more values. Note STEP number
			22166,		100,	//	63		one extra value so we do not run off the table
		}
	};

#endif	// USE_ELEVATION_LINEAR_DRIVE

#ifdef NODEF
		{
		#warning Elevation Motor Gearbox 575:1
		#warning using 2 ticks per motor revolution
		// Ticks    %dc			step
		// "Speed”,	in percent
			0,			0,		//  0		NOTE: this row does NOT come from the spreadsheet
			14119,		28,		//	1		Spreadsheet data starts HERE
			11898,		32,		//	2
			10476,		35,		//	3
			9466,		38,		//	4
			8701,		40,		//	5
			8096,		43,		//	6
			7602,		45,		//	7
			7188,		47,		//	8
			6836,		49,		//	9
			6530,		51,		//	10
			6263,		53,		//	11
			6025,		55,		//	12		half speed, limit for SLEW, starting point for stall recovery
			5813,		57,		//	13
			5622,		58,		//	14
			5448,		60,		//	15
			5290,		61,		//	16
			5145,		63,		//	17
			5011,		64,		//	18
			4886,		66,		//	19
			4771,		67,		//	20
			4664,		69,		//	21
			4563,		70,		//	22
			4469,		71,		//	23
			4380,		72,		//	24
			4297,		74,		//	25
			4218,		75,		//	26
			4143,		76,		//	27
			4072,		77,		//	28
			4004,		79,		//	29
			3940,		80,		//	30
			3879,		81,		//	31
			3821,		82,		//	32
			3765,		83,		//	33
			3711,		84,		//	34
			3660,		85,		//	35
			3611,		86,		//	36
			3564,		87,		//	37
			3518,		88,		//	38
			3474,		89,		//	39
			3432,		90,		//	40
			3392,		91,		//	41
			3352,		92,		//	42
			3314,		93,		//	43
			3278,		94,		//	44
			3242,		95,		//	45
			3208,		96,		//	46
			3175,		97,		//	47
			3142,		98,		//	48
			3111,		99,		//	49
			3081,		100,	//	50
			3051,		101		//	51		one extra value so we do not run off the table
		}
	};
#endif

#else	// NOT DEFINE_GLOBALS
	GLOBAL ARRAY MOTION_PROFILE_TBL_TYPE MotionProfileSpeedAndPWM[NUM_MOTORS][(MOTION_PROFILE_TABLE_LEN * 2) + 2];
#endif	// NOT DEFINE_GLOBALS


// end of MotionProfile.h
