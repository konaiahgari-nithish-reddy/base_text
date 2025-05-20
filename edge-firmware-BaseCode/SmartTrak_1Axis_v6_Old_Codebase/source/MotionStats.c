// *************************************************************************************************
//									M o t i o n S t a t s . C
// *************************************************************************************************
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Control statistics for debugging and analysis
//
//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
//	Include Files
//-------------------------------------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include <string.h>				// Microchip string functions
								// see hlpC18.chm online help for so-called documentation
#include <ctype.h>				// tolower()

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"
#include "EventFlags.h"			// event flag definitions and globals

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting

#ifdef USE_MMA8452Q_INCLINOMETER
	#include "mma845x.h"              // MMA845xQ definitions
	#include "Inclinometer.h"
#endif

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Static Function Prototypes
//-------------------------------------------------------------------------------------------------------

// *************************************************************************************************
//									M o t i o n S t a t s
// *************************************************************************************************

// MotionPhaseFSM() ST_MTN_PHASE_FSM_ACCELERATE_FWD or ST_MTN_PHASE_FSM_ACCELERATE_REV calls PWM_SetConfig(), which in turn calls this function
// NOTE: SetMotionLimits() is called BEFORE this is called (not a good arrangement?)

void Init_MotionStats(enum tagMotors eMotor)
{

	pgMotionStats[eMotor].ePWMDirection = PWM_DIR_STOPPED;

	pgMotionStats[eMotor].lStartingPosition = CurrentPosition_Read(eMotor);
	pgMotionStats[eMotor].lEndingPosition = 0;
	pgMotionStats[eMotor].lDistance = 0;

	pgMotionStats[eMotor].bPWM_AccelDutyCycleMin = 100;					// init to maximum value so that it will be overwritten
	pgMotionStats[eMotor].bPWM_AccelDutyCycleMax = 0;					// init to minimum value
	pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMin = 0;			// init to minimum value
	pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMax = 0;			// init to minimum value
	pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMin = 100;			// init to maximum value
	pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMax= 0;			// init to minimum value
	pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMin = 0;	// init to minimum value
	pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMax = 0;	// init to minimum value
	pgMotionStats[eMotor].bPWM_DecelDutyCycleMin = 100;					// init to maximum value
	pgMotionStats[eMotor].bPWM_DecelDutyCycleMax = 0;					// init to minimum value

	pgMotionStats[eMotor].bPWMAccelMotionIndex = 0;
	pgMotionStats[eMotor].bPWMDecelMotionIndex = 0;
	pgMotionStats[eMotor].bSpeedlMotionIndex = 0;

	// NOTE: speed values are count of Tcy ticks, so SMALLER is FASTER
	pgMotionStats[eMotor].MinimumSpeed = 0L;								// slower is larger
	pgMotionStats[eMotor].MaximumSpeed = MOTION_PROFILE_MAX_SPEED_VALUE;	// faster is smaller
	#ifdef CALC_AVERAGE_SPEED
		pgMotionStats[eMotor].wAverageSpeed = 0;
	#endif
	pgMotionStats[eMotor].LastMotionStallTimer = 0;

	pgMotionStats[eMotor].wMSI_AccelerationCount = 0;
	pgMotionStats[eMotor].ulMSI_ConstantSpeedCount = 0L;
	pgMotionStats[eMotor].wMSI_DecelerationCount = 0;
	pgMotionStats[eMotor].wMSI_MinimumPWMCount = 0;
	pgMotionStats[eMotor].wMSI_CoastCount = 0;
	pgMotionStats[eMotor].ulMSI_TotalCount = 0L;

	pgMotionStats[eMotor].wAccelerationPWMAdjustmentCount = 0;
	pgMotionStats[eMotor].ulConstantSpeedPWMAdjustmentCount = 0;

	pgMotionStats[eMotor].efStartingEvent = 0;
	pgMotionStats[eMotor].efEndingEvent = 0;

	pgMotionStats[eMotor].efAccelerationMotionEvents = 0;
	pgMotionStats[eMotor].efConstantSpeedMotionEvents = 0;
	pgMotionStats[eMotor].efDecelerationMotionEvents = 0;
	pgMotionStats[eMotor].efMinimumPWMMotionEvents = 0;
	pgMotionStats[eMotor].efCoastingMotionEvents = 0;

	pgwMSI_AccelerationCtr[eMotor] = 0;									// MSI ticks for each of the motion phases
	pgulMSI_ConstantSpeedCtr[eMotor] = 0;
	pgwMSI_DecelerationCtr[eMotor] = 0;
	pgwMSI_MinimumPWMCtr[eMotor] = 0;
	pgwMSI_CoastCtr[eMotor] = 0;
	pgulMoveTotalMSICtr[eMotor] = 0;

	pgMotionStats[eMotor].lStartTime = 0L;								// timer snapshots
	pgMotionStats[eMotor].lAccelerationEndTime = 0L;
	pgMotionStats[eMotor].lConstantSpeedEndTime = 0L;
	pgMotionStats[eMotor].lDecelerationEndTime = 0L;
	pgMotionStats[eMotor].lMinimumPWMEndTime = 0L;
	pgMotionStats[eMotor].lCoastEndTime = 0L;

	// see MotionPhaseFSM.h
	pgMotionStats[eMotor].eMoveType = MOVE_NONE;

	#ifdef USE_INCLINOMETER_FEEDBACK
		pgMotionStats[eMotor].fStartingAngle = pgfStartingAngle[eMotor];				// initialized by SetMotionLimits()
		pgMotionStats[eMotor].fMoveDistanceDegrees = pgfMoveDistanceDegrees[eMotor];	// requested move distance
		pgMotionStats[eMotor].fEndingAngle = 0.0;
		pgMotionStats[eMotor].fFinalAngle = 0.0;
	#endif

}

// copy global data to MotionStats structure, and clean up some values..

void Finish_MotionStats(enum tagMotors eMotor)
{

	pgMotionStats[eMotor].ePWMDirection = pgePWMDirection[eMotor];
	pgMotionStats[eMotor].lEndingPosition = CurrentPosition_Read(eMotor);
	pgMotionStats[eMotor].lDistance = pgMotionStats[eMotor].lEndingPosition - pgMotionStats[eMotor].lStartingPosition;

	pgMotionStats[eMotor].LastMotionStallTimer = pgLastMotionStallTimer[eMotor];
	#ifdef CALC_AVERAGE_SPEED
		pgMotionStats[eMotor].wAverageSpeed = pgwAverageSpeed[eMotor];
	#endif

	pgMotionStats[eMotor].wMSI_AccelerationCount = pgwMSI_AccelerationCtr[eMotor];
	pgMotionStats[eMotor].ulMSI_ConstantSpeedCount = pgulMSI_ConstantSpeedCtr[eMotor];
	pgMotionStats[eMotor].wMSI_DecelerationCount = pgwMSI_DecelerationCtr[eMotor];
	pgMotionStats[eMotor].wMSI_MinimumPWMCount = pgwMSI_MinimumPWMCtr[eMotor];
	pgMotionStats[eMotor].wMSI_CoastCount = pgwMSI_CoastCtr[eMotor];
	pgMotionStats[eMotor].ulMSI_TotalCount = pgulMoveTotalMSICtr[eMotor];

//	pgMotionStats[eMotor].bSpeedlMotionIndex = pgbMotionProfileSpeedIndex;

	// adjust times; some segments may not occur
	if (pgMotionStats[eMotor].lConstantSpeedEndTime IS 0L)
		pgMotionStats[eMotor].lConstantSpeedEndTime = pgMotionStats[eMotor].lAccelerationEndTime;

	if (pgMotionStats[eMotor].lDecelerationEndTime IS 0L)
		pgMotionStats[eMotor].lDecelerationEndTime = pgMotionStats[eMotor].lConstantSpeedEndTime;

	if (pgMotionStats[eMotor].lMinimumPWMEndTime IS 0L)
		pgMotionStats[eMotor].lMinimumPWMEndTime = pgMotionStats[eMotor].lDecelerationEndTime;

	if (pgMotionStats[eMotor].lCoastEndTime IS 0L)
		pgMotionStats[eMotor].lCoastEndTime  = pgMotionStats[eMotor].lMinimumPWMEndTime;

	#ifdef USE_INCLINOMETER_FEEDBACK
		// pgMotionStats[eMotor].fEndingAngle is set by MotionSensor_Tick(), while still in motion

		// now that motion has stopped, clear and restart Inclinometer averaging
		IGNORE_RETURN_VALUE Init_InclinometerSampleAveraging();

		// this is STOPPED, final angle
		pgMotionStats[eMotor].fFinalAngle = CurrentAverageAngleDegrees_Read(eMotor);

	#endif

}


#ifdef MOTION_ERROR_TABLE
	// initialize the Motion Error (speed) table, stores speed, speed error, etc on each tick, for debugging/analysis ONLY
	void ClearMotionErrorTable(void)
	{
		memset((void *)&pgMotionProfileSpeedErrorTbl, 0x00, (size_t)(sizeof(MOTION_PROFILE_SPEED_ERROR_TBL) * SPEED_ERROR_TABLE_LEN));
		
		// initialize table index
		pgwMotionProfileSpeedErrorIndex = 0;
	}
#endif


#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
	// initialize the Polar Axis Move table; stores orientation at the end of each move, for debugging/analysis ONLY
	void ClearPolarAxisMoveTable(void)
	{
		BYTE i;				// choice of type limits POLAR_AXIS_MOVE_TBL_LEN to <= 255


		for(i = 0; i < POLAR_AXIS_MOVE_TBL_LEN; i++)
		{
			pgPolarAxisMoveTbl[i].cHours = 0;
			pgPolarAxisMoveTbl[i].fAzimuthAngleDegrees = 0.0;
			pgPolarAxisMoveTbl[i].fModuleTiltAngleDegrees = 0.0;
			pgPolarAxisMoveTbl[i].fBackTrackAngleDegrees = 0.0;
			pgPolarAxisMoveTbl[i].fSunElevationAngleDegrees = 0.0;
		}
	}
#endif	// defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)


// end of MotionStats.c

