// *************************************************************************************************
//										M o t i o n S t a t s . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Tracking Globals, function prototypes, definitions
//
// *************************************************************************************************


//----------------------------------------------------------------------
//	Function Prototypes
//----------------------------------------------------------------------

void Init_MotionStats(enum tagMotors eMotor);
void Finish_MotionStats(enum tagMotors eMotor);

#ifdef MOTION_ERROR_TABLE
	void ClearMotionErrorTable(void);
#endif

#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
	void ClearPolarAxisMoveTable(void);
#endif	// defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)


//----------------------------------------------------------------------
//	Definitions
//----------------------------------------------------------------------

// structure to keep track of all information about single move
// this structure can be dumped via the serial menu
// we **MAY** want to keep an array of these, to view multiple moves..
typedef struct
	{
	enum 	tagPWMDirection	ePWMDirection;

	INT32	lStartingPosition;
	INT32	lEndingPosition;
	INT32	lDistance;

	BYTE	bPWM_AccelDutyCycleMin;
	BYTE	bPWM_AccelDutyCycleMax;
	INT8S	cPWM_AccelDutyCycleCorrectionMin;
	INT8S	cPWM_AccelDutyCycleCorrectionMax;
	BYTE	bPWM_ConstantSpeedDutyCycleMin;
	BYTE	bPWM_ConstantSpeedDutyCycleMax;
	INT8S	cPWM_ConstantSpeedDutyCycleCorrectionMin;
	INT8S	cPWM_ConstantSpeedDutyCycleCorrectionMax;
	BYTE	bPWM_DecelDutyCycleMax;
	BYTE	bPWM_DecelDutyCycleMin;

	BYTE	bPWMAccelMotionIndex;
	BYTE	bPWMDecelMotionIndex;
	BYTE	bSpeedlMotionIndex;

	MOTION_PROFILE_SPEED_TYPE	MinimumSpeed;
	MOTION_PROFILE_SPEED_TYPE	MaximumSpeed;
	#ifdef CALC_AVERAGE_SPEED
		WORD	wAverageSpeed;
	#endif
	MOTION_PROFILE_SPEED_TYPE	LastMotionStallTimer;

	WORD	wMSI_AccelerationCount;				// MSI ticks for each of the motion phases (NOT used as counters; just to capture count at end of move)
	UINT32	ulMSI_ConstantSpeedCount;			// need to be changed to UINT32 to handle MoveToXXXStow
	WORD	wMSI_DecelerationCount;
	WORD	wMSI_MinimumPWMCount;
	WORD	wMSI_CoastCount;
	UINT32	ulMSI_TotalCount;

	WORD	wAccelerationPWMAdjustmentCount;	// count of duty cycle corrections
	UINT32	ulConstantSpeedPWMAdjustmentCount;	// count of duty cycle corrections

	// note: these are just 16 bit numbers, so the total times will come out wrong if the TOTAL run time > 2.27s
	// all need to be changed to UINT32 to handle MoveToXXXStow; this may be handled by separate upper WORD values.. which will need flags to keep track of which is active
	UINT32	lStartTime;							// timer snapshots
	UINT32	lAccelerationEndTime;
	UINT32	lConstantSpeedEndTime;
	UINT32	lDecelerationEndTime;
	UINT32	lMinimumPWMEndTime;
	UINT32	lCoastEndTime;

	EVENTFLAGS	efStartingEvent;
	EVENTFLAGS	efEndingEvent;

	EVENTFLAGS	efAccelerationMotionEvents;
	EVENTFLAGS	efConstantSpeedMotionEvents;
	EVENTFLAGS	efDecelerationMotionEvents;
	EVENTFLAGS	efMinimumPWMMotionEvents;
	EVENTFLAGS	efCoastingMotionEvents;

	// see MotionPhaseFSM.h:
	enum tagMoveTypes eMoveType;

	#ifdef USE_INCLINOMETER_FEEDBACK
		float	fStartingAngle;
		float	fMoveDistanceDegrees;
		float	fEndingAngle;
		float	fFinalAngle;
	#endif


} MOTION_STATS, *PTR_MOTION_STATS;


#ifdef MOTION_ERROR_TABLE
	typedef struct
	{
		UINT8	bMotionProfileIndex;
		MOTION_PROFILE_SPEED_TYPE		ExpectedSpeed;
		MOTION_PROFILE_SPEED_TYPE		MeasuredSpeed;
		MOTION_PROFILE_SPEED_ERR_TYPE	SpeedError;
		INT8	cPWMCorrection;
		UINT8	bPWM;
		#ifdef USE_INCLINOMETER_FEEDBACK
			MOTION_PROFILE_DISTANCE_TYPE	LastMoveDistanceTicks;
		#endif

	} MOTION_PROFILE_SPEED_ERROR_TBL, PTR_MOTION_PROFILE_SPEED_ERROR_TBL;

	#define	SPEED_ERROR_TABLE_LEN	512				// table size is large enough for a complete ?.0 degree move
#endif

#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
	// Polar Axis Move table; stores orientation at the end of each move, for debugging/analysis ONLY
	typedef struct
	{
		UINT8	cHours;							// time, incomplete but adequate for debugging

		// all of these are in degrees
		float	fAzimuthAngleDegrees;			// final calculated value
		float	fModuleTiltAngleDegrees;		// calculated value
		float	fBackTrackAngleDegrees;			// calculated value
		float	fSunElevationAngleDegrees;		// calculated value, not used elsewhere?

	} POLAR_AXIS_MOVE_TBL, PTR_POLAR_AXIS_MOVE_TBL;

	#define	POLAR_AXIS_MOVE_TBL_LEN		100		// approximately 2 days of data for Single Polar Axis implementation
#endif

//----------------------------------------------------------------------
//	Global Variables
//----------------------------------------------------------------------
#ifdef DEFINE_GLOBALS
	GLOBAL MOTION_STATS pgMotionStats[NUM_MOTORS];

	#ifdef MOTION_ERROR_TABLE
		// note: this array is never READ by software; it exists only to allow viewing with the debugger
		// data type must match the MotionProfileSpeedAndPWM[][] data type, UINT16 or UINT32
		//lint -esym(552,pgMotionProfileSpeedErrorTbl)		error 552: (Warning -- Symbol 'pgMotionProfileSpeedErrorTbl' not accessed)
//		GLOBAL ARRAY MOTION_PROFILE_TBL_TYPE pgsMotionProfileSpeedError[SPEED_ERROR_TABLE_SIZE];
								// create direct index into Speed Error table (implemented as a simple array, but used as 4 columns)
		GLOBAL ARRAY MOTION_PROFILE_SPEED_ERROR_TBL pgMotionProfileSpeedErrorTbl[SPEED_ERROR_TABLE_LEN];
		GLOBAL_INIT	WORD pgwMotionProfileSpeedErrorIndex = 0;
	#endif
	#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
		GLOBAL ARRAY POLAR_AXIS_MOVE_TBL pgPolarAxisMoveTbl[POLAR_AXIS_MOVE_TBL_LEN];
		GLOBAL_INIT	BYTE pgbPolarAxisMoveTblIndex = 0;
	#endif
#else
	GLOBAL MOTION_STATS pgMotionStats[];

	#ifdef MOTION_ERROR_TABLE
		//lint -esym(552,pgsMotionProfileSpeedError)		error 552: (Warning -- Symbol 'pgsMotionProfileSpeedError' not accessed)
		GLOBAL ARRAY MOTION_PROFILE_SPEED_ERROR_TBL pgMotionProfileSpeedErrorTbl[];
		GLOBAL WORD pgwMotionProfileSpeedErrorIndex;
	#endif
	#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
		GLOBAL ARRAY POLAR_AXIS_MOVE_TBL pgPolarAxisMoveTbl[];
		GLOBAL BYTE pgbPolarAxisMoveTblIndex;
	#endif
#endif

// end of MotionStats.h
