// *************************************************************************************************
//										M o t i o n P r o f i l e . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Profile Table
//
// *************************************************************************************************

// Motion Profile table limits, see below


// *************************************
//			Table Length
// *************************************
#define	AZ_MOTION_PROFILE_TABLE_LEN				(148 + 1)							// manually taken from MotionProfileSpeedAndPWM[MOTOR_AZIMUTH][] table below

#if defined(USE_ELEVATION_SLEW_DRIVE)
	#define	EL_MOTION_PROFILE_TABLE_LEN			(60 + 1)							// manually taken from MotionProfileSpeedAndPWM[MOTOR_ELEVATION][] table below
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_MOTION_PROFILE_TABLE_LEN			(41 + 1)							// manually taken from MotionProfileSpeedAndPWM[MOTOR_ELEVATION][] table below
#else
	#error Elevation Drive Type must be specified
#endif

// MOTION_PROFILE_TABLE_LEN is the longer of the two tables; the shorter table will have uninitialized data that is not used
#define	MOTION_PROFILE_TABLE_LEN			AZ_MOTION_PROFILE_TABLE_LEN		// longer of two tables


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

#define	AZ_SLEW_MAX_SPEED_INDEX					67								// table index for 0.75 maximum speed, see table below

#if defined(USE_ELEVATION_SLEW_DRIVE)
	#define	EL_SLEW_MAX_SPEED_INDEX				33								// table index for 0.75 maximum speed, see table below
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_SLEW_MAX_SPEED_INDEX				23								// table index for 0.75 maximum speed, see table below
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
	#define	EL_STALL_RECOVERY_MIN_INDEX			5			// one third speed, PWM 36%
	#define	EL_STALL_RECOVERY_MAX_INDEX			35			// 75% speed
	#define	EL_STALL_RECOVERY_INCREMENT			5			// 6 steps, 5 to 35
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_STALL_RECOVERY_MIN_INDEX			5			// one third speed, PWM 36%
	#define	EL_STALL_RECOVERY_MAX_INDEX			23			// 75% speed
	#define	EL_STALL_RECOVERY_INCREMENT			3			// 6 steps, 5 to 23  (figure out this manually, from above two values
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


#ifdef USE_SINGLE_POLAR_AXIS
	#define	AZ_RUN_MINIMUM_MOVE_TICKS			(AZ_MSI_TICKS_PER_DEGREE * 2)		// single polar axis move is generally 2 degrees
#else
	#define	AZ_RUN_MINIMUM_MOVE_TICKS			5							// minimum move size in ticks
#endif

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


#define	EL_LINEAR_DRIVE_SIM_TICK_COUNT		4				// approx ratio of MSI ticks, slew drive to linear actuator drive
															// based on current spreadsheet values, we want to use only 1 in 13 MSI ticks
															//		calculation: slew drive: 2 ticks per motor revolution, linear drive: 0.1586 ticks per motor revolution

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

//#if defined(USE_ELEVATION_SLEW_DRIVE)
	typedef	UINT16	MOTION_PROFILE_TBL_TYPE;		// speed values all fit in unsigned 16 bit value
	typedef	UINT16	MOTION_PROFILE_SPEED_TYPE;		// speed values all fit in unsigned 16 bit value
	#define MOTION_PROFILE_MAX_SPEED_VALUE	0xFFFF	// maximum value to fit in unsigned 16 bit
	typedef	INT16S	MOTION_PROFILE_SPEED_ERR_TYPE;	// speed error values all fit in signed 16 bit value
	typedef INT32S	MOTION_PROFILE_DISTANCE_TYPE;	// distance value (for 278:73:1 slew drive in particular) requires signed 32 bit value
//#elif defined(USE_ELEVATION_LINEAR_DRIVE)
//	typedef	UINT32	MOTION_PROFILE_TBL_TYPE;		// slowest speed values reqiure a 32 bit value
//	typedef	UINT32	MOTION_PROFILE_SPEED_TYPE;		// speed values all fit in unsigned 32 bit value
//	#define MOTION_PROFILE_MAX_SPEED_VALUE	0xFFFFFFFF	// maximum value to fit in unsigned 16 bit
//	typedef	INT32S	MOTION_PROFILE_SPEED_ERR_TYPE;	// speed error values all fit in signed 32 bit value
//#endif


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
		// this table is based on the Slew Drive unit currently used in the field
		#warning Azimuth Motor Gearbox 236:1 + Slew Gearbox 73:1, 2 ticks per motor rev, 10MHz PBCLK, 24V
		// Ticks       %dc		  step
		// "Speed”,	  in percent
			0,			0,		//  0	NOTE: this row does NOT come from the spreadsheet
			2941,		28,		//	1	Starting PWM value 75.3mS @ 39.1KHz
			2760,		29,		//	2
			2608,		31,		//	3
			2479,		32,		//	4
			2367,		33,		//	5
			2269,		34,		//	6
			2182,		35,		//	7
			2105,		36,		//	8
			2035,		37,		//	9
			1972,		38,		//	10
			1914,		39,		//	11
			1861,		40,		//	12
			1813,		40,		//	13
			1768,		41,		//	14
			1726,		42,		//	15
			1687,		43,		//	16
			1650,		44,		//	17
			1616,		44,		//	18
			1584,		45,		//	19	 1 Second
			1553,		46,		//	20
			1525,		47,		//	21
			1498,		47,		//	22
			1472,		48,		//	23
			1447,		49,		//	24
			1424,		49,		//	25
			1402,		50,		//	26	50% of maximum speed  starting point for stall recovery
			1381,		51,		//	27
			1361,		51,		//	28
			1341,		52,		//	29
			1323,		52,		//	30
			1305,		53,		//	31
			1288,		54,		//	32
			1271,		54,		//	33
			1255,		55,		//	34
			1240,		55,		//	35
			1225,		56,		//	36
			1211,		57,		//	37
			1197,		57,		//	38
			1184,		58,		//	39
			1171,		58,		//	40
			1159,		59,		//	41
			1147,		59,		//	42
			1135,		60,		//	43
			1124,		60,		//	44
			1113,		61,		//	45
			1102,		61,		//	46
			1092,		62,		//	47
			1082,		62,		//	48
			1072,		63,		//	49
			1062,		63,		//	50
			1053,		64,		//	51
			1044,		64,		//	52
			1035,		65,		//	53
			1026,		65,		//	54
			1018,		66,		//	55
			1010,		66,		//	56
			1002,		67,		//	57
			994,		67,		//	58
			986,		68,		//	59
			979,		68,		//	60
			972,		69,		//	61
			964,		69,		//	62
			957,		69,		//	63
			951,		70,		//	64
			944,		70,		//	65
			937,		71,		//	66
			931,		71,		//	67	75% of maximum speed
			925,		72,		//	68
			919,		72,		//	69
			913,		72,		//	70
			907,		73,		//	71
			901,		73,		//	72
			895,		74,		//	73
			890,		74,		//	74
			884,		75,		//	75
			879,		75,		//	76
			873,		75,		//	77
			868,		76,		//	78
			863,		76,		//	79
			858,		77,		//	80
			853,		77,		//	81
			848,		77,		//	82
			844,		78,		//	83
			839,		78,		//	84
			834,		79,		//	85
			830,		79,		//	86
			825,		79,		//	87
			821,		80,		//	88
			817,		80,		//	89
			812,		80,		//	90
			808,		81,		//	91
			804,		81,		//	92
			800,		82,		//	93
			796,		82,		//	94
			792,		82,		//	95
			788,		83,		//	96
			784,		83,		//	97
			781,		83,		//	98
			777,		84,		//	99
			773,		84,		//	100
			770,		84,		//	101
			766,		85,		//	102
			763,		85,		//	103
			759,		86,		//	104
			756,		86,		//	105
			752,		86,		//	106
			749,		87,		//	107
			746,		87,		//	108
			742,		87,		//	109
			739,		88,		//	110
			736,		88,		//	111
			733,		88,		//	112
			730,		89,		//	113
			727,		89,		//	114
			724,		89,		//	115
			721,		90,		//	116
			718,		90,		//	117
			715,		90,		//	118
			712,		91,		//	119
			709,		91,		//	120
			707,		91,		//	121
			704,		92,		//	122
			701,		92,		//	123
			698,		92,		//	124
			696,		93,		//	125
			693,		93,		//	126
			690,		93,		//	127
			688,		94,		//	128
			685,		94,		//	129
			683,		94,		//	130
			680,		94,		//	131
			678,		95,		//	132
			675,		95,		//	133
			673,		95,		//	134
			671,		96,		//	135
			668,		96,		//	136
			666,		96,		//	137
			664,		97,		//	138
			661,		97,		//	139
			659,		97,		//	140
			657,		98,		//	141
			655,		98,		//	142
			652,		98,		//	143
			650,		98,		//	144
			648,		99,		//	145
			646,		99,		//	146
			644,		99,		//	147
			642,		100,	//	148	 100%, no need for data below
			640,		100,	//	149  16.4mS @ 39.1KHz
		},

#ifdef USE_ELEVATION_SLEW_DRIVE
		{
		#warning Elevation Motor Gearbox 575:1, 2 ticks per motor rev, 10MHz PBCLK, 24V
		// Ticks    %dc			step
		// "Speed”,	in percent
			0,			0,		//  0	NOTE: this row does NOT come from the spreadsheet
			2941,		28,		//	1	Starting PWM value  75.3mS @ 39.1KHz
			2541,		31,		//	2
			2269,		34,		//	3
			2069,		36,		//	4
			1914,		39,		//	5	One third speed
			1790,		41,		//	6
			1687,		43,		//	7
			1600,		45,		//	8
			1525,		47,		//	9
			1459,		48,		//	10
			1402,		50,		//	11
			1351,		52,		//	12
			1305,		53,		//	13
			1263,		55,		//	14	Half Speed
			1225,		56,		//	15
			1191,		57,		//	16
			1159,		59,		//	17
			1129,		60,		//	18
			1102,		61,		//	19
			1077,		63,		//	20
			1053,		64,		//	21
			1031,		65,		//	22
			1010,		66,		//	23
			990,		67,		//	24
			972,		69,		//	25
			954,		70,		//	26
			937,		71,		//	27
			922,		72,		//	28	 1 Second
			907,		73,		//	29
			892,		74,		//	30
			879,		75,		//	31
			866,		76,		//	32
			853,		77,		//	33	75% Speed
			841,		78,		//	34
			830,		79,		//	35
			819,		80,		//	36
			808,		81,		//	37
			798,		82,		//	38
			788,		83,		//	39
			779,		84,		//	40
			770,		84,		//	41
			761,		85,		//	42
			752,		86,		//	43
			744,		87,		//	44
			736,		88,		//	45
			728,		89,		//	46
			721,		90,		//	47
			714,		90,		//	48
			707,		91,		//	49
			700,		92,		//	50
			693,		93,		//	51
			687,		94,		//	52
			680,		94,		//	53
			674,		95,		//	54
			668,		96,		//	55
			663,		97,		//	56
			657,		98,		//	57
			651,		98,		//	58
			646,		99,		//	59
			641,		100,	//	60	100% PWM value, no need for more values. Note STEP number
			636,		100		//	61  16.3mS @ 39.1KHz

		}
	};
#endif	// USE_ELEVATION_SLEW_DRIVE

#ifdef USE_ELEVATION_LINEAR_DRIVE
		{
		#warning Elevation Linear Actuator 58:1, 46 ticks per Linear Inch, 10MHz PBCLK, 24V
		// Ticks    %dc			step
		// "Speed”,	in percent
			0,			0,		//  0	NOTE: this row does NOT come from the spreadsheet
			29552	,	28	,	//	1	Starting PWM value
			24094	,	33	,	//	2
			20851	,	36	,	//	3
			18642	,	40	,	//	4	One Third Speed , for stall
			17012	,	43	,	//	5
			15747	,	46	,	//	6
			14728	,	48	,	//	7
			13884	,	51	,	//	8
			13170	,	53	,	//	9
			12556	,	55	,	//	10	Half Speed
			12021	,	57	,	//	11
			11549	,	59	,	//	12
			11128	,	61	,	//	13
			10750	,	63	,	//	14
			10409	,	65	,	//	15
			10097	,	67	,	//	16
			9813	,	68	,	//	17
			9551	,	70	,	//	18
			9309	,	71	,	//	19
			9084	,	73	,	//	20
			8875	,	75	,	//	21
			8680	,	76	,	//	22
			8497	,	78	,	//	23	 75% of full speed
			8325	,	79	,	//	24
			8163	,	80	,	//	25
			8011	,	82	,	//	26
			7866	,	83	,	//	27
			7729	,	84	,	//	28
			7599	,	86	,	//	29
			7476	,	87	,	//	30
			7358	,	88	,	//	31
			7246	,	90	,	//	32
			7138	,	91	,	//	33
			7035	,	92	,	//	34	 10 Seconds
			6937	,	93	,	//	35
			6843	,	94	,	//	36
			6752	,	96	,	//	37
			6665	,	97	,	//	38
			6581	,	98	,	//	39
			6500	,	99	,	//	40
			6422	,	100	,	//	41	 100%, no need for data below
			6347	,	101		//	42
		}
	};

#endif	// USE_ELEVATION_LINEAR_DRIVE


#else	// NOT DEFINE_GLOBALS
	GLOBAL ARRAY MOTION_PROFILE_TBL_TYPE MotionProfileSpeedAndPWM[NUM_MOTORS][(MOTION_PROFILE_TABLE_LEN * 2) + 2];
#endif	// NOT DEFINE_GLOBALS


// end of MotionProfile.h
