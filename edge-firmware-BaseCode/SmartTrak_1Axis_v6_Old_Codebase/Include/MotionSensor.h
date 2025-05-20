// *************************************************************************************************
//									M o t i o n S e n s o r . h
// *************************************************************************************************
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Sensor function definitions, global variables, for 2 axis
//
// *************************************************************************************************


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagMotionSensorErrors
{
	MTN_SENSOR_ERROR_NONE = MTN_SENSOR_ERROR_BASE,
	MTN_SENSOR_ERROR_UNEXPECTED_TICK,			// 1 unexpected sensor tick event
	MTN_SENSOR_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	MTN_SENSOR_ERROR_INVALID_INPUT_LEVEL,		// 3 sensor input level is LOW
	MTN_SENSOR_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	MTN_SENSOR_ERROR_INVALID_MOTION_PHASE,		// 5 motion phase not valid during interrupt
	MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK,	// 6 motion type not valid during interrupt
	MTN_SENSOR_ERROR_COASTING_INT,				// 7 interrupt generated during coasting
	MTN_SENSOR_ERROR_FOREGROUND_TICK_OVERRUN,	// 8 foreground tick processing overrun
	MTN_SENSOR_ERROR_MSI_COUNT_DONE_OVERRUN,	// 9 event overrun
	MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_INT,	// A motion type not valid during interrupt
	MTN_SENSOR_ERROR_NO_EVENT_GENERATED,		// B no event generated during MotionSensor_Tick()
	MTN_SENSOR_ERROR_INVALID_AXIS,				// C not a valid axis or motor ID
	MTN_SENSOR_ERROR_UNEXPECTED_INT	,			// D unexpected sensor tick event
	MTN_SENSOR_ERROR_INVALID_TABLE_INDEX,		// E motion table index out of range

	MTN_SENSOR_UNPROCESSED_EVENT = MTN_SENSOR_ERROR_BASE + 0x0F
};

#define TIMER3_CLK_PRESCALE		256			// timer 3 is used for Input Capture Motion Sensor Timing


//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

// these are file-global variables, defined in EITHER MotionSensor.c OR Inclinometer.c

#if (defined(USE_HALL_SENSOR_FEEDBACK) && defined(MOTIONSENSOR_C)) || (defined(USE_INCLINOMETER_FEEDBACK) AND defined(INCLINOMETER_C))

//	FILE_GLOBAL_INIT INT32 fglCurrentPosition[NUM_MOTORS] = {0L, 0L};

//	PRIVATE_INIT WORD	fgwLastTimerValue[NUM_MOTORS] = {0xFFFF, 0xFFFF};

	#ifdef CALC_AVERAGE_SPEED
		PRIVATE_INIT WORD	fgwSumOfSpeeds[NUM_MOTORS] = {0, 0};
		PRIVATE_INIT WORD	fgwSampleCtr[NUM_MOTORS] = {0, 0};
		PRIVATE_INIT WORD	fgwLastSpeed[NUM_MOTORS] = {0, 0};
	#endif
	PRIVATE_INIT MOTION_PROFILE_SPEED_TYPE	fguExpectedSpeed[NUM_MOTORS] = {0, 0};
#endif

//-------------------------------------------------------------------------------------------------------
// Function Prototypes
//-------------------------------------------------------------------------------------------------------

// MotionSensor.c and Inclinometer.c have separate versions of this function
void MotionSensor_Init(void);


// MotionSensor.c and Inclinometer.c have separate versions of these functions
void MotionSensor_EnableInt(enum tagAxis eAxis);
void MotionSensor_DisableInt(enum tagAxis eAxis);
void MotionSensor_Tick(enum tagMotors eMotor);

UINT16 GetMotionSensorState(void);

// Speed and Position set/get functions in MotionSensor.c are used for both Hall Effect Sensor and Inclinometer feedback
MOTION_PROFILE_SPEED_TYPE CurrentSpeed_Read(enum tagMotors eMotor);
void CurrentSpeed_Write(enum tagMotors eMotor, MOTION_PROFILE_SPEED_TYPE NewSpeed);

void CurrentPosition_Clear(enum tagMotors eMotor);
INT32 CurrentPosition_Read(enum tagMotors eMotor);
float CurrentAverageAngleDegrees_Read(enum tagMotors eMotor);
void CurrentPosition_Set(enum tagMotors eMotor, INT32 lNewPosition);
void CurrentMechanicalOrientation_Read(SmartTrakOrientation *ptrOrientation);
void CurrentLocalOrientation_Read(SmartTrakOrientation *ptrOrientation);

//unsigned int Azimuth_MotionSensor_Read(void);
//unsigned int Elevation_MotionSensor_Read(void);
//BYTE GetAzimuthMotionSensorState( void );
//BYTE GetElevationMotionSensorState( void );

#define MOTION_SENSOR_EDGE_DELAY_US		100

#ifdef USE_FEEDBACK_SIMULATOR
	BOOL MS_SetSimulatorTickCtr(enum tagMotors eMotor);
#endif

//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------
#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

// NOTE: the WORD and short types used here may not be large enough if the ticks-per-degree values increase
// WORD (w) is unsigned 16 bit
// short (s) is signed 16 bit
#ifdef DEFINE_GLOBALS
	GLOBAL_INIT MOTION_PROFILE_SPEED_TYPE		pguCurrentSpeed[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT MOTION_PROFILE_SPEED_ERR_TYPE	pgsSpeedError[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT INT32							fglCurrentPosition[NUM_MOTORS] = {0L, 0L};
	GLOBAL_INIT WORD							fgwLastTimerValue[NUM_MOTORS] = {0xFFFF, 0xFFFF};
	GLOBAL_INIT INT32							fglPreviousPosition[NUM_MOTORS] = {0L, 0L};
	#ifdef CALC_AVERAGE_SPEED
		GLOBAL_INIT MOTION_PROFILE_SPEED_TYPE		pguAverageSpeed[NUM_MOTORS] = {0, 0};
		GLOBAL_INIT MOTION_PROFILE_SPEED_ERR_TYPE	pgsErrorFromAverage[NUM_MOTORS] = {0, 0};
		GLOBAL_INIT MOTION_PROFILE_SPEED_ERR_TYPE	pgsErrorFromSample[NUM_MOTORS] = {0, 0};
	#endif
	GLOBAL_INIT	WORD	pgwStateMSICtr[NUM_MOTORS] = {0, 0};		// motion sensor ticks in motion FSM state ctr; signed because we may miss counting down to 0
	GLOBAL_INIT	UINT32	pgulMoveTotalMSICtr[NUM_MOTORS] = {0L, 0L};	// motion sensor ticks in move ctr, does not need to be UINT32 for present system
	GLOBAL_INIT	WORD	pgwStateMSILimit[NUM_MOTORS] = {0, 0};
#elif defined (DEFINE_EXTERNS)
	GLOBAL MOTION_PROFILE_SPEED_TYPE			pguCurrentSpeed[];
	GLOBAL MOTION_PROFILE_SPEED_ERR_TYPE		pgsSpeedError[];
	GLOBAL INT32								fglCurrentPosition[];
	GLOBAL WORD									fgwLastTimerValue[];
	GLOBAL INT32								fglPreviousPosition[];
	#ifdef CALC_AVERAGE_SPEED
		GLOBAL MOTION_PROFILE_SPEED_TYPE		pguAverageSpeed[];
		GLOBAL MOTION_PROFILE_SPEED_ERR_TYPE	pgsErrorFromAverage[];
		GLOBAL MOTION_PROFILE_SPEED_ERR_TYPE	pgsErrorFromSample[];
	#endif
	GLOBAL WORD			pgwStateMSICtr[];
	GLOBAL UINT32		pgulMoveTotalMSICtr[];
	GLOBAL WORD			pgwStateMSILimit[];
#endif


// end of Encoder.h
