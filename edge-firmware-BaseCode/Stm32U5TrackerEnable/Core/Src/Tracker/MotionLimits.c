// *************************************************************************************************
//										M o t i o n L i m i t s . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Limit calculations
//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
//	Include Files
//------------------------------------------------------------ -------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include <math.h>				// fabs()

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
#include "MoveSequenceFSM.h"	// for PTR_SPA_MOVE_INFO

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

#include "CoordTranslate.h"		// coordinate translation functions
#include "LEDs.h"				// LED display handler function definition, used for stall recovery states

//#include "Stubs.h"

#ifdef DEFINE_GLOBALS
#error "DEFINE_GLOBALS not expected here"
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagMotionLimitsErrors
{
	MTN_LIMIT_ERROR_NONE = MTN_LIMIT_ERROR_BASE,
	MTN_LIMIT_ERROR_UNEXPECTED_TICK,		// 1 unexpected timer tick event
	MTN_LIMIT_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	MTN_LIMIT_ERROR_INVALID_STATE,			// 3 not a valid state
	MTN_LIMIT_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	MTN_LIMIT_ERROR_INVALID_MOVE,			// 5 not a valid move type
	MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE,		// 6 move not possible from current position
	MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE_FWD,	// 7 move not possible from current position
	MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE_REV,	// 8 move not possible from current position
	MTN_LIMIT_ERROR_TRUNCATE_MOVE,			// 9 truncate move distance to not go past soft limits
	MTN_LIMIT_ERROR_POSITION_ERROR,			// A current position is out of range
	MTN_LIMIT_ERROR_PWM_DECREMENT_ZERO,		// B PWM decrement value is 0
	MTN_LIMIT_ERROR_INVALID_DIRECTION,		// C direction flag is not set correctly
	MTN_LIMIT_ERROR_INVALID_AXIS,			// D not a valid axis or motor ID
	MTN_LIMIT_ERROR_CANNOT_REVERSE_RECOVERY	// E cannot reverse for stall recovery

};

//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

// position/orientation values (Ticks) are signed. They may not fit in an INT16, so they need to be INT32
// size limitation is the 236:73:1 Slew drive
FILE_GLOBAL_INIT INT32	fglStartingPosition[NUM_MOTORS] = {0, 0};
FILE_GLOBAL_INIT INT32	fglEndingPosition[NUM_MOTORS] = {0, 0};

//	externals/globals used in SetMotionLimits():
//	Flash Constants
//		ptrRAM_SystemParameters->fAZ_SoftLimit_Forward
//		ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse
//		ptrRAM_SystemParameters->fEL_SoftLimit_Forward
//		ptrRAM_SystemParameters->fEL_SoftLimit_Reverse
//		ptrRAM_SystemParameters->fAZ_Offset
//		ptrRAM_SystemParameters->fEL_Offset
//
//	Move Setup, must be initialized before call to SetMotionLimits()
//		pglMoveDistanceTicks[eMotor]	input, requested move distance
//		pgfMoveDistanceDegrees[eMotor]	Inclinometer feedback ONLY, requested move distance, in addition to pglMoveDistanceTicks
//
//	Motion Profile for Move Type (Run, Slew, Stall Recovery)
//		pgbMotionProfileIndexIncrement[eMotor]
//		pgbMotionProfileIndexDecrement[eMotor]
//		pgbMotionProfileIndexMin[eMotor]
//		pgbMotionProfileIndexMax[eMotor]
//		pgbMotionProfileSpeedIndex[eMotor]
//
//	Move Breakdown for Motion Profile
//		pgwMSIAccelerationCount[eMotor]
//		pgulMSIConstantSpeedCount[eMotor]
//		pgwMSIDecelerationCount[eMotor]
//		pgwStateMSICtr[eMotor]
//
//	PWM Setup
//		pgbPWMDutyCycleMin[eMotor]
//		pgePWMDirection[eMotor]
//		pgeStallRecoveryPWMConfig[eMotor]


// pgfMS_StepDistanceDegrees (may be used externally)


// pgulMSIConstantSpeedCount

BOOL SetMotionLimits(enum tagMotors eMotor, enum tagMoveTypes eMove)
{

	// position/orientation values are signed. They may not fit in an INT16, so they need to be INT32
	UINT32 ulDistanceTicks;				// total distance, in ticks
	UINT32 ulMissingDistanceTicks;		// initially missing distance in ticks, used for constant speed correction
	INT32 lMaxForwardPosition = 0;
	INT32 lMaxReversePosition = 0;
#ifdef USE_INCLINOMETER_FEEDBACK
	float fMaxForwardAngleDegrees = 0.0;
	float fMaxReverseAngleDegrees = 0.0;
#endif

	// *******************************************
	//		Get Current Position or Angle
	// *******************************************
	// get current MECHANICAL position (does NOT account for offsets)
	fglStartingPosition[eMotor] = CurrentPosition_Read(eMotor);
#ifdef USE_INCLINOMETER_FEEDBACK
	pgfStartingAngle[eMotor] = CurrentAverageAngleDegrees_Read(eMotor);
#endif

	// *******************************************
	//		Calculate Travel Limits
	// *******************************************
	// set RUN motion absolute boundaries, DOES NOT apply to SLEW_TO_END
	// these values set the movement boundaries and keep us away from the hard stops
	// ==>> these **should** be the end of travel detected positions, minus 2.5 degrees for mechanical clearance
	if (eMotor IS MOTOR_AZIMUTH)
	{
		lMaxForwardPosition = ConvertDegreesToMSITicks(ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset, AXIS_AZIMUTH);
		lMaxReversePosition = ConvertDegreesToMSITicks(ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset, AXIS_AZIMUTH);

#ifdef USE_INCLINOMETER_FEEDBACK
		fMaxForwardAngleDegrees = ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset, AXIS_AZIMUTH;
		fMaxReverseAngleDegrees = ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset, AXIS_AZIMUTH;
#endif

	}
	else if(eMotor IS MOTOR_ELEVATION)
	{
#ifdef USE_INCLINOMETER_FEEDBACK
		RuntimeError(MTN_LIMIT_ERROR_INVALID_AXIS);
		return FALSE;
#endif
		lMaxForwardPosition = ConvertDegreesToMSITicks(ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset, AXIS_ELEVATION);
		lMaxReversePosition = ConvertDegreesToMSITicks(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset, AXIS_ELEVATION);
	}
	else
	{
		// unlikely error
		RuntimeError(MTN_LIMIT_ERROR_INVALID_AXIS);
#ifdef USE_DYNAMIC_LEDS
		SetLEDs(LEDS_NO_MOVE);
#endif
		return FALSE;
	}

	// *******************************************
	//		Set Move Distance, check for 0
	// *******************************************
	// make sure we do not run into the end-of-travel, which is actually 2.5 degrees from the limit switches
	// code here allows for two options, determined by config.h #defines USE_PREVENT_MOVE_PAST_LIMIT, USE_TRUNCATE_MOVE_PAST_LIMIT

	// we are NOT calculating final end positions at this time; just checking for a possible move

	// get the distance to move (ticks) as a POSITIVE number, for consistent math
	ulDistanceTicks = (UINT32)ABS(pglMoveDistanceTicks[eMotor]);

	// check for 0 length move prior to any truncation of move distance
#ifdef USE_INCLINOMETER_FEEDBACK
	if (fabs(pgfMoveDistanceDegrees[eMotor]) < 0.2)
#else
		if (ulDistanceTicks IS 0L)
#endif
		{
			RuntimeError(MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE);
#ifdef USE_DYNAMIC_LEDS
			SetLEDs(LEDS_NO_MOVE);
#endif
			return FALSE;
		}

	// *******************************************
	//			Set Motion Limits
	// *******************************************
	switch (eMove)
	{
	// REVERSE == Left/Down (negative position numbers)
	case MOVE_RUN_REV:							// reverse, full speed
		// make sure we are not somehow already past the end-of-travel
		if (fglStartingPosition[eMotor] < lMaxReversePosition)
		{
			RuntimeError(MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE);
#ifdef USE_DYNAMIC_LEDS
			SetLEDs(LEDS_NO_MOVE);
#endif
			return FALSE;
		}

		// calculate ending position
		fglEndingPosition[eMotor] = fglStartingPosition[eMotor] - pglMoveDistanceTicks[eMotor];
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfEndingPositionDegrees[eMotor] = pgfStartingAngle[eMotor] - pgfMoveDistanceDegrees[eMotor];	// pgfMoveDistanceDegrees is set externally by caller
#endif

		// if we are too close to the end-stop to allow a complete move, the move is not allowed
		if (fglEndingPosition[eMotor] < (lMaxReversePosition))
		{
#if defined(USE_PREVENT_MOVE_PAST_LIMIT)
			// too close to the end-stop, cannot move
			RuntimeError(MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE_REV);
#ifdef USE_DYNAMIC_LEDS
			SetLEDs(LEDS_NO_MOVE);
#endif
			return FALSE;
#elif defined (USE_TRUNCATE_MOVE_PAST_LIMIT)
			// truncate move distance to prevent moving past soft limit
			RuntimeError(MTN_LIMIT_ERROR_TRUNCATE_MOVE);
			// calculate new, truncated move distance
			fglEndingPosition[eMotor] = lMaxReversePosition;
			pglMoveDistanceTicks[eMotor] = fglStartingPosition[eMotor] - fglEndingPosition[eMotor];
#ifdef USE_INCLINOMETER_FEEDBACK
			pgfEndingPositionDegrees[eMotor] = fMaxReverseAngleDegrees;
			pgfMoveDistanceDegrees[eMotor] = pgfStartingAngle[eMotor] - pgfEndingPositionDegrees[eMotor];
#endif
#endif
		}
		break;


		// FORWARD == Right/Up (positive position numbers)
	case MOVE_RUN_FWD:								// forward, full speed
		// make sure we are not somehow already past the end-of-travel
		if (fglStartingPosition[eMotor] > lMaxForwardPosition)
		{
			RuntimeError(MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE);
#ifdef USE_DYNAMIC_LEDS
			SetLEDs(LEDS_NO_MOVE);
#endif
			return FALSE;
		}
		// calculate ending position for a short move
		fglEndingPosition[eMotor] = fglStartingPosition[eMotor] + pglMoveDistanceTicks[eMotor];
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfEndingPositionDegrees[eMotor] = pgfStartingAngle[eMotor] + pgfMoveDistanceDegrees[eMotor];	// pgfMoveDistanceDegrees is set externally by caller
#endif

		// if we are too close to the end-stop to allow a complete  move, the move is not allowed
		if (fglEndingPosition[eMotor] > (lMaxForwardPosition))
		{
#if defined(USE_PREVENT_MOVE_PAST_LIMIT)
			// too close to the end-stop, cannot move
			RuntimeError(MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE_FWD);
#ifdef USE_DYNAMIC_LEDS
			SetLEDs(LEDS_NO_MOVE);
#endif
			return FALSE;
#elif defined (USE_TRUNCATE_MOVE_PAST_LIMIT)
			// truncate move distance to prevent moving past soft limit
			RuntimeError(MTN_LIMIT_ERROR_TRUNCATE_MOVE);
			// calculate new, truncated move distance
			fglEndingPosition[eMotor] = lMaxForwardPosition;
			pglMoveDistanceTicks[eMotor] = fglEndingPosition[eMotor] - fglStartingPosition[eMotor];
#ifdef USE_INCLINOMETER_FEEDBACK
			pgfEndingPositionDegrees[eMotor] = fMaxForwardAngleDegrees;
			pgfMoveDistanceDegrees[eMotor] = pgfEndingPositionDegrees[eMotor] - pgfStartingAngle[eMotor];
#endif

#endif
		}
		break;

	case MOVE_SLEW_REV_TO_END:							// move REVERSE (left/down) to STOP, no bounds check because motion is limited by EOT detection
		break;

	case MOVE_SLEW_FWD_TO_END:							// move FORWARD (right/up) to STOP, no bounds check because motion is limited by EOT detection
		break;

#ifdef USE_MOVE_CENTER_CMD
	case MOVE_CENTER:									// move to Center
		// we ALWAYS have enough room to move to the center!
		break;
#endif

	case MOVE_STALL_RECOVERY:							// no bounds check
		break;

	case MOVE_NONE:										// no move, stopped
	case MOVE_COMPLETE:									// not meaningful, but prevents error 788: (Info -- enum constant 'tagMoveTypes::MOVE_COMPLETE' not used within defaulted switch)
	default:
		RuntimeError(MTN_LIMIT_ERROR_INVALID_MOVE);
#ifdef USE_DYNAMIC_LEDS
		SetLEDs(LEDS_NO_MOVE);
#endif
		return FALSE;
	}

#if defined (USE_TRUNCATE_MOVE_PAST_LIMIT)
	// re-establish move distance, which MAY have been adjusted by above code
	ulDistanceTicks = (UINT32)ABS(pglMoveDistanceTicks[eMotor]);

	// check for zero length move after truncation
#ifdef USE_INCLINOMETER_FEEDBACK
	if (fabs(pgfMoveDistanceDegrees[eMotor]) < 0.2)
#else
		if (ulDistanceTicks IS 0L)
#endif
		{
			RuntimeError(MTN_LIMIT_ERROR_NO_MOVE_POSSIBLE);
#ifdef USE_DYNAMIC_LEDS
			SetLEDs(LEDS_NO_MOVE);
#endif
			return FALSE;
		}
#endif

	// *******************************************
	//			Set Phase Lengths
	// *******************************************
	// we have enough travel available for the requested move (pglMoveDistanceTicks[eMotor])

	switch (eMove)
	{
	// accelerate to maximum speed, move distance is finite, specified by caller
	case MOVE_RUN_FWD:									// move FORWARD (right/up)
	case MOVE_RUN_REV:									// move REVERSE (left/down)
		// determine distance of unknown length move - assume we are moving to the stop
		// pgnEndingPosition was calculated above
		// ulDistanceTicks is the absolute value of input variable pglMoveDistanceTicks[eMotor]

		// look up the first PWM value from the Motion Profile table
		pgbPWMDutyCycleMin[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(MIN_PWM_INDEX * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

		// set motion profile control values
		pgbMotionProfileIndexIncrement[eMotor] = RUN_MOTION_PROFILE_INDEX_INCREMENT;
		pgbMotionProfileIndexDecrement[eMotor] = RUN_MOTION_PROFILE_INDEX_DECREMENT;
		pgbMotionProfileIndexMin[eMotor] = RUN_MOTION_PROFILE_INDEX_MIN;
		pgbMotionProfileIndexMax[eMotor] = RUN_MOTION_PROFILE_INDEX_MAX(eMotor);
		pgbMotionProfileSpeedIndex[eMotor] = RUN_MOTION_PROFILE_INDEX_MIN;					// set initial Motion Profile index

		// set motion phase lengths (we have already checked to make sure there is room a minimal move)
		// determine if motion will require a constant speed phase (total distance longer than acceleration + deceleration)
		if (ulDistanceTicks >  (UINT32)(MSI_RUN_ACCELERATION_COUNT(eMotor) + MSI_RUN_DECELERATION_COUNT(eMotor)))	// <sek> 18 Aug 13
		{
			// constant speed phase required
			pgwMSIAccelerationCount[eMotor] = MSI_RUN_ACCELERATION_COUNT(eMotor);
			pgulMSIConstantSpeedCount[eMotor] = ulDistanceTicks - (MSI_RUN_ACCELERATION_COUNT(eMotor) + MSI_RUN_DECELERATION_COUNT(eMotor));
			pgwMSIDecelerationCount[eMotor] = MSI_RUN_DECELERATION_COUNT(eMotor);
		}
		else
		{
			// no constant speed phase, so we just have to distribute the acceleration and deceleration phases
			pgwMSIAccelerationCount[eMotor] = (UINT16)(RUN_ACCEL_TO_DECEL_RATIO * (ulDistanceTicks / (RUN_ACCEL_TO_DECEL_RATIO + 1)));	// <sek> 18 Aug 13
			pgulMSIConstantSpeedCount[eMotor] = 0;
			pgwMSIDecelerationCount[eMotor] = (UINT16)(ulDistanceTicks / (RUN_ACCEL_TO_DECEL_RATIO + 1));	// <sek> 18 Aug 13

			// the above math may (actually probably will!) result in a move SHORTER than requested. if the move will be under-length
			// add a constant speed phase to make up the difference
			ulMissingDistanceTicks = ulDistanceTicks - (UINT32)((UINT32)pgwMSIAccelerationCount[eMotor] + (UINT32)pgwMSIDecelerationCount[eMotor]);

			if (ulMissingDistanceTicks IS_NOT 0L)
			{
				pgulMSIConstantSpeedCount[eMotor] = ulMissingDistanceTicks;		// constant speed phase corrects total distance
			}

			// adjust maximum duty cycle to account for the shortened move.. simply look up from the motion profile table
			// unused pgbPWMDutyCycleMax = MotionProfileSpeedAndPWM[(pgwMSIAccelerationCount * MOVE_MOTION_PROFILE_INDEX_INCREMENT * 2) + PWM_VALUE_OFFSET];
		}

		// this can ONLY occur before the start of motion, so initialize the state MSI tick limit
		pgwStateMSICtr[eMotor] = 0;								// restart state encoder tick counter
		pgwStateMSILimit[eMotor] = pgwMSIAccelerationCount[eMotor];
		break;

#ifdef NOTDEF
	case MOVE_REV_STOP:											// moving Reverse (left/down) orderly STOP
	case MOVE_FWD_STOP:											// moving Forward (right/up) orderly STOP
		// adjust MSI counts to bring motion to an orderly stop
		pgwMSIAccelerationCount = 0;
		pgwMSIRunCount = 0;
		// Deceleration should be based on whatever the actual acceleration was..
		pgwMSIDecelerationCount = pgwMSI_AccelerationCtr / (MSI_ACCEL_TO_DECEL_RATIO + 1);

		// no matter where this occurs, we immediately change state to DECELERATE, so re-initialize the state MSI tick limit
		pgwStateMSILimit = pgwMSIDecelerationCount;

		// this is ONLY used to end MOVE_LT, MOVE_RT, so there is no need to change the PWM duty cycle settings
		break;
#endif

#ifdef USE_MOVE_CENTER_CMD
		// NOTE: 04 Sep 13 this is old, untested code, left here because it may be useful in the future
	case MOVE_CENTER:											// move to Center
		// determine distance of unknown length move - assume we are moving to the CENTER (0)
		ulDistanceTicks = (WORD)ABS(pgnStartingPosition);

		// look up the PWM limits for a SLEW from the Motion Profile table  (Max PWM may be adjusted below)
		pgbPWMDutyCycleMin = (BYTE)MotionProfileSpeedAndPWM[(SLEW_MOTION_PROFILE_INDEX_MIN * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];
		// unused pgbPWMDutyCycleMax = MotionProfileSpeedAndPWM[(SLEW_MOTION_PROFILE_INDEX_MAX(eMotor) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

		// set motion profile control values same as a SLEW
		pgbMotionProfileIndexIncrement[eMotor] = RUN_MOTION_PROFILE_INDEX_INCREMENT;
		pgbMotionProfileIndexDecrement[eMotor] = RUN_MOTION_PROFILE_INDEX_DECREMENT;
		pgbMotionProfileIndexMin[eMotor] = RUN_MOTION_PROFILE_INDEX_MIN;
		pgbMotionProfileIndexMax[eMotor] = RUN_MOTION_PROFILE_INDEX_MAX(eMotor);
		pgbMotionProfileSpeedIndex[eMotor] = RUN_MOTION_PROFILE_INDEX_MIN;					// set initial Motion Profile index

		// determine if motion will require a constant speed RUN phase (total distance longer than acceleration + deceleration)
		if (ulDistanceTicks >  (MSI_SLEW_ACCELERATION_COUNT(eMotor) + MSI_SLEW_DECELERATION_COUNT(eMotor)))
		{
			// run phase required
			pgwMSIAccelerationCount[eMotor] = MSI_SLEW_ACCELERATION_COUNT(eMotor);
			pgulMSIConstantSpeedCount[eMotor] = ulDistanceTicks - (MSI_SLEW_ACCELERATION_COUNT(eMotor) + MSI_SLEW_DECELERATION_COUNT(eMotor));
			pgwMSIDecelerationCount[eMotor] = MSI_SLEW_DECELERATION_COUNT(eMotor);
		}
		else
		{
			// no run phase, so we just have to distribute the acceleration and deceleration phases
			// MOVE_CENTER is the same as SLEW, which is currently setup for equal acceleration and deceleration
			pgwMSIAccelerationCount[eMotor] = ulDistanceTicks / 2;
			pgulMSIConstantSpeedCount[eMotor] = 0;
			pgwMSIDecelerationCount[eMotor] = ulDistanceTicks - pgwMSIAccelerationCount;

			// adjust maximum duty cycle to account for the shortened move
			// unused pgbPWMDutyCycleMax = (BYTE)MotionProfileSpeedAndPWM[(pgwMSIAccelerationCount * SLEW_MOTION_PROFILE_INDEX_INCREMENT * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];
		}

		// this can ONLY occur before the start of motion, so initialize the state MSI tick limit
		pgwStateMSICtr[eMotor] = 0;									// restart state encoder tick counter
		pgwStateMSILimit[eMotor] = pgwMSIAccelerationCount[eMotor];
		break;
#endif	// USE_MOVE_CENTER_CMD


	case MOVE_SLEW_FWD_TO_END:									// move maximum distance right/up until end of travel sense
	case MOVE_SLEW_REV_TO_END:									// move maximum distance left/down until end of travel sense
		// half speed, maximum distance constant spped phase - a MOVE_XX_TO_END is used to find the end stops
		pgwMSIAccelerationCount[eMotor] = MSI_SLEW_ACCELERATION_COUNT(eMotor);
		pgulMSIConstantSpeedCount[eMotor] = MSI_SLEW_CONSTANT_SP_COUNT(eMotor);
		pgwMSIDecelerationCount[eMotor] = MSI_SLEW_ACCELERATION_COUNT(eMotor);

		// look up the PWM limits for a SLEW from the Motion Profile table
		pgbPWMDutyCycleMin[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(SLEW_MOTION_PROFILE_INDEX_MIN * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];
		// unused pgbPWMDutyCycleMax = (BYTE)MotionProfileSpeedAndPWM[(SLEW_MOTION_PROFILE_INDEX_MAX(eMotor) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

		// set motion profile control values same as a SLEW
		pgbMotionProfileIndexIncrement[eMotor] = SLEW_MOTION_PROFILE_INDEX_INCREMENT;
		pgbMotionProfileIndexDecrement[eMotor] = SLEW_MOTION_PROFILE_INDEX_DECREMENT;
		pgbMotionProfileIndexMin[eMotor] = SLEW_MOTION_PROFILE_INDEX_MIN;
		pgbMotionProfileIndexMax[eMotor] = SLEW_MOTION_PROFILE_INDEX_MAX(eMotor);
		pgbMotionProfileSpeedIndex[eMotor] = SLEW_MOTION_PROFILE_INDEX_MIN;					// set initial Motion Profile index

		// this can ONLY occur before the start of motion, so initialize the state MSI tick limit
		pgwStateMSICtr[eMotor] = 0;									// restart state encoder tick counter
		pgwStateMSILimit[eMotor] = pgwMSIAccelerationCount[eMotor];
		break;

	case MOVE_STALL_RECOVERY:								// very short reverse move to recover from a stall
		// stall recovery does NOT use the motion profile table
		pgwMSIAccelerationCount[eMotor] = MSI_STALL_RECOVERY_ACCELERATION_COUNT(eMotor);
		pgulMSIConstantSpeedCount[eMotor] = MSI_STALL_RECOVERY_CONSTANT_SP_COUNT(eMotor);
		pgwMSIDecelerationCount[eMotor] = MSI_STALL_RECOVERY_DECELERATION_COUNT(eMotor);

		// look up the PWM limits for a SLEW from the Motion Profile table
		pgbPWMDutyCycleMin[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(STALL_RECOVERY_MOTION_PROFILE_INDEX_MIN(eMotor) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];
		// unused pgbPWMDutyCycleMax = (BYTE)MotionProfileSpeedAndPWM[(SLEW_MOTION_PROFILE_INDEX_MAX(eMotor) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

		// set motion profile control values same as a SLEW
		pgbMotionProfileIndexIncrement[eMotor] = STALL_RECOVERY_MOTION_PROFILE_INDEX_INCREMENT(eMotor);
		pgbMotionProfileIndexDecrement[eMotor] = STALL_RECOVERY_MOTION_PROFILE_INDEX_DECREMENT(eMotor);
		pgbMotionProfileIndexMin[eMotor] = STALL_RECOVERY_MOTION_PROFILE_INDEX_MIN(eMotor);
		pgbMotionProfileIndexMax[eMotor] = STALL_RECOVERY_MOTION_PROFILE_INDEX_MAX(eMotor);
		pgbMotionProfileSpeedIndex[eMotor] = STALL_RECOVERY_MOTION_PROFILE_INDEX_MIN(eMotor);				// set initial Motion Profile index

		// this can ONLY occur before the start of (the stall recovery) motion, so initialize the state MSI tick limit
		pgwStateMSICtr[eMotor] = 0;									// restart state encoder tick counter
		// set the State MSI Limit to just <2>, so we exit as soon as there is some real motion
		// (a single MSI tick by itself does not necessarily indicate much motion!)
		pgwStateMSILimit[eMotor] = pgwMSIAccelerationCount[eMotor];

		// set motion profile control values
		// unused pgbPWMDutyCycleIncrement = PWM_STALL_RECOVERY_INCREMENT;
		// unused pgbPWMDutyCycleDecrement = PWM_STALL_RECOVERY_DECREMENT;		// <== never actually used!

		// figure out direction for stall recovery
		// we are primarily interested in stall recovery when hitting the physical limit switches at the end of travel,
		// but should be able to handle stalls that may occur anywhere
		// if we are within 2 short moves of the end of travel, stall recovery must be towards 0, regardless of the previous direction
		// set new PWM bridge direction, reverse of previous direction
		if (pgePWMDirection[eMotor] IS PWM_DIR_FORWARD)
		{
			if (fglStartingPosition[eMotor] > MSI_STALL_RECOVERY_REVERSE_LIMIT(eMotor))			// ==> change to system parameter?
			{
				// we are far enough from the reverse limit to allow reverse recovery
				pgeStallRecoveryPWMConfig[eMotor] = PWM_CONFIG_REVERSE;
			}
			else
			{
				// too close to reverse limit to allow reverse recovery, we MUST move forward
				pgeStallRecoveryPWMConfig[eMotor] = PWM_CONFIG_FORWARD;
				RuntimeError(MTN_LIMIT_ERROR_CANNOT_REVERSE_RECOVERY);
			}
		}
		else if (pgePWMDirection[eMotor] IS PWM_DIR_REVERSE)
		{
			if (fglStartingPosition[eMotor] < MSI_STALL_RECOVERY_FORWARD_LIMIT(eMotor))			// ==> change to system parameter?
			{
				// we are far enough from the forward limit to allow forward recovery
				pgeStallRecoveryPWMConfig[eMotor] = PWM_CONFIG_FORWARD;
			}
			else
			{
				// too close to forward limit to allow forward recovery, we MUST move reverse
				pgeStallRecoveryPWMConfig[eMotor] = PWM_CONFIG_REVERSE;
				RuntimeError(MTN_LIMIT_ERROR_CANNOT_REVERSE_RECOVERY);
			}
		}
		else
		{
			// not a valid direction
			RuntimeError(MTN_LIMIT_ERROR_INVALID_DIRECTION);
		}
		break;


	case MOVE_NONE:
	case MOVE_COMPLETE:								// not meaningful, but prevents error 788: (Info -- enum constant 'tagMoveTypes::MOVE_COMPLETE' not used within defaulted switch)
	default:
		RuntimeError(MTN_LIMIT_ERROR_INVALID_MOVE);
		return FALSE;
	}

	// bounds check Duty Cycle Decrement value, which is calculated - and may be 0
	/* unused
	if (pgbPWMDutyCycleDecrement IS 0)
		{
		pgbPWMDutyCycleDecrement = 1;
		RuntimeError(MTN_LIMIT_ERROR_PWM_DECREMENT_ZERO);
		}
	 */


	return TRUE;


}

// end of MotionLimits.c
