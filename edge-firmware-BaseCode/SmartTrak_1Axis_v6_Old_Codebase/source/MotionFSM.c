// *************************************************************************************************
//									M o t i o n F S M . c
// *************************************************************************************************
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Finite State Machine, for 2 Axis
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
#include <legacy\int_3xx_4xx_legacy.h>	// required for Input Capture interrupt handlers
//lint +e14

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

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

#include "LEDs.h"				// LED display handler function definition, used for stall recovery states

#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagMotionFSMErrors
{
	MTN_FSM_ERROR_NONE = MTN_FSM_ERROR_BASE,
	MTN_FSM_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	MTN_FSM_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	MTN_FSM_ERROR_INVALID_STATE,			// 3 not a valid state
	MTN_FSM_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	MTN_FSM_ERROR_DUTY_CYCLE_NOT_MIN,		// 5 duty cycle is not expected minimum value
	MTN_FSM_ERROR_STOP_OVERRUN,				// 6 command to User FSM overrun (may occur due to different execution rates)
	MTN_FSM_ERROR_ACCELERATION_DONE_OVERRUN,// 7 command to User FSM overrun (may occur due to different execution rates)
	MTN_FSM_ERROR_BELOW_MINIMUM_PWM,		// 8 PWM has adjusted below minimum value
	MTN_FSM_ERROR_TOO_FAST_MAX_PWM,			// 9 Too Fast AND at maximum PWM
	MTN_FSM_ERROR_MAXIMUM_PWM,				// A just unexpected maximum PWM
	MTN_FSM_ERROR_INVALID_DIRECTION,		// B unknown previous PWM direction
	MTN_FSM_ERROR_NON_PWM_UPDATE_TICK,		// C non PWM tick during Acceleration or Deceleration
	MTN_FSM_ERROR_TOO_MANY_TICKS,			// D too many calls into FSM during Accel or Decel
	MTN_FSM_ERROR_ABOVE_MAXIMUM_PWM,		// E PWM has adjusted above maximum value
	MTN_FSM_ERROR_UNPROCESSED_EVENT = MTN_FSM_ERROR_BASE + 0x0F

};


//-------------------------------------------------------------------------------------------------------
// File Globals - Static Variables
//-------------------------------------------------------------------------------------------------------

PRIVATE_INIT BYTE bMotionFSMSequenceCtr[NUM_MOTORS] = {0, 0};				// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus[NUM_MOTORS] = {SUBSTATE_NOT_DONE, SUBSTATE_NOT_DONE};	// substate status flag

PRIVATE_INIT BYTE fgbAdjustedDutyCycle[NUM_MOTORS] = {0, 0};						// current PWM duty cycle

//lint -esym(551,fgefMotionEventsCopy)			error 551: (Warning -- Symbol 'fgefMotionEventsCopy' (line 111, file source\MotionFSM.c) not accessed)
PRIVATE_INIT EVENTFLAGS fgefMotionEventsCopy[NUM_MOTORS] = {0, 0};			// copy of event flags, for debugging, never accessed by software

//lint -esym(552,efUnprocessedMotionEvents)		error 552: (Warning -- Symbol 'efUnprocessedMotionEvents' not accessed)
/*PRIVATE_INIT*/ EVENTFLAGS efUnprocessedMotionEvents[NUM_MOTORS] = {0, 0};	// used for debugging purposes ONLY, to track unprocessed events

PRIVATE_INIT BOOL bStallHasOccured[NUM_MOTORS] = {FALSE, FALSE};			// flag to capture occurence of stall without acting on it.

PRIVATE_INIT BYTE fgbMotionProfilePWMIndex[NUM_MOTORS] = {0, 0};			// index into motion profile table for reading PWM values; may be out of sync with index for speed values

PRIVATE_INIT BYTE fgbSoftStallDelayCtr = 0;									// counter to increase delay in softstall before changing directions ==> need separate variables for each axis

//-------------------------------------------------------------------------------------------------------
// Static Function Prototypes
//-------------------------------------------------------------------------------------------------------

// see MotionFSM.h for tagMotionStates definition
// enum tagMotionStates eMotionState[NUM_MOTORS] = {ST_MOTION_STOPPED, ST_MOTION_STOPPED};
enum tagMotionStates eMotionState[NUM_MOTORS] = {ST_MOTION_INIT, ST_MOTION_INIT};

// this is an array of pointers to state descriptor strings
// NOTE: this array must exactly track the above enum in order for it to make any sense!
FILE_GLOBAL ARRAY const char *pstrMotionStateText[] =
	{
	"Init",
	"Stopped",
	"Acc",
	"Acc +PWM",
	"Acc -PWM",
	"Constant Speed",
	"Constant +PWM",
	"Constant -PWM",
	"Dec",
	"Min PWM",
	"Coast",
	"Soft Stall",
	"Hard Stall",
	""
	};


const char *GetMotionStateString(enum tagMotors eMotor)
{
	// return pointer to state descriptor string
	// note that we are not doing any bounds checking..
	return (pstrMotionStateText[eMotionState[eMotor]]);

}

enum tagMotionStates GetMotionState(enum tagMotors eMotor)
{
	return(eMotionState[eMotor]);
}


/*
	**NOTE** this information dates to early in the project, and needs to be updated to reflect current implementation and reality..
	FSM Parameters:
		Initial PWM width
		Acceleration in PWM width change per timer interrupt
		Number of MSI ticks during acceleration (may not be enough to reach max speed)
		(Needs to account for current travel limit)
		Final PWM width at end of acceleration
		Number of MSI ticks during constant speed (may be 0)
		(Needs to account for current travel limit)
		Deceleration in PWM width change per timer interrupt
		Number of MSI ticks during deceleration
		(Needs to account for current travel limit)
		Minimum PWM width at end of deceleration

	External FSM Events
		New Motion Command
*/

// *************************************************************************************************
//								S y n c h r o n i z a t i o n 
// *************************************************************************************************

// these functions exist to avoid the problem of synchronizing external events with move commands.
// (specifically serial Menu Commands and the Move Sequencer)
// When a process external to the MotionFSM (specifically serial Menu Commands and the Move Sequencer) starts a move,
// it also needs to know when the move is complete. The state of eMotionState is NOT an adequate indicator because
// it stays ST_MOTION_STOPPED until the motion actually starts, and overall system timing may allow the external process
// to query eMotionState before the motion starts - and interpret it as motion complete.

// so the idea here is that the external process (specifically serial Menu Commands and the Move Sequencer) calls SetMotionStarted()
// to indicate that a move is in process, setting the fgbIsMotionStarted flag to TRUE BEFORE setting an efMotionEvents flag to actually START the motion

// When the MotionFSM enters ST_MOTION_STOPPED from some other state, the FIRST substate of ST_MOTION_STOPPED calls ClearMotionStarted() JUST ONCE
// to clear the fgbIsMotionStarted flag to FALSE.

FILE_GLOBAL_INIT BOOL fgbIsMotionStarted[NUM_MOTORS] = {FALSE, FALSE};

void SetMotionStarted(enum tagMotors eMotor)
	{
	fgbIsMotionStarted[eMotor] = TRUE;
	}

void ClearMotionStarted(enum tagMotors eMotor)
	{
	fgbIsMotionStarted[eMotor] = FALSE;
	}

BOOL IsMotionComplete(enum tagMotors eMotor)
	{
	if (fgbIsMotionStarted[eMotor])
		// motion has started, so it cannot be complete
		return FALSE;
	else
		return TRUE;

	}

// not useful/not necessary? only for a HARD_STALL recovery?
void ResetMotionFSM(void)
{

	DisableMotorDrive();									// disable motor driver ICs (MC33926)

	// reinitialize all persistent variables to allow restart after a stall
	eMotionState[MOTOR_AZIMUTH] = ST_MOTION_INIT;
	eMotionState[MOTOR_ELEVATION] = ST_MOTION_INIT;

	// first entry into MotionFSM() will initialize bMotionFSMSequenceCtr, fSubStateStatus

	fgbAdjustedDutyCycle[MOTOR_AZIMUTH] = 0;				// current PWM duty cycle
	fgbAdjustedDutyCycle[MOTOR_ELEVATION] = 0;

	fgefMotionEventsCopy[MOTOR_AZIMUTH] = 0;				// copy of event flags, for debugging, never accessed by software
	fgefMotionEventsCopy[MOTOR_ELEVATION] = 0;

	efUnprocessedMotionEvents[MOTOR_AZIMUTH] = 0;			// used for debugging purposes ONLY, to track unprocessed events
	efUnprocessedMotionEvents[MOTOR_ELEVATION] = 0;

	bStallHasOccured[MOTOR_AZIMUTH] = FALSE;				// flag to capture occurence of stall without acting on it.
	bStallHasOccured[MOTOR_ELEVATION] = FALSE;

	fgbMotionProfilePWMIndex[MOTOR_AZIMUTH] = 0;			// index into motion profile table for reading PWM values; may be out of sync with index for speed values
	fgbMotionProfilePWMIndex[MOTOR_ELEVATION] = 0;

	fgbSoftStallDelayCtr = 0;								// counter to increase delay in softstall before changing directions (temp kludge?)

	// PWM_SetConfig() is called upon entry to ST_MOTION_STOPPED
	// pgeMotionPhase is initialized  upon entry to ST_MOTION_STOPPED
	// pgeMotionType is initialized  upon entry to ST_MOTION_STOPPED
	//PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_STOPPED);	// stop both axis
	//PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);
	//pgeMotionPhase[MOTOR_AZIMUTH] = PHASE_STOPPED;
	//pgeMotionPhase[MOTOR_ELEVATION] = PHASE_STOPPED;
	//pgeMotionType[MOTOR_AZIMUTH] = MOTION_STOPPED;
	//pgeMotionType[MOTOR_ELEVATION] = MOTION_STOPPED;

	ClearMotionStarted(MOTOR_AZIMUTH);
	ClearMotionStarted(MOTOR_ELEVATION);

}

// *************************************************************************************************
//								M o t i o n F S M ( )
// *************************************************************************************************

// this Finite State Machine is called repeatedly on timer ticks and input events
// execution is flow-through-and-exit; we never stay in this routine
// delays are handled by external timers and subsequent calls to this routine

void MotionFSM(enum tagMotors eMotor)
{

	EVENTFLAGS  efMotionEventsUponEntry;				// used for debugging purposes ONLY, keeps a copy of Motion Event flags at entry to Motion FSM
	EVENTFLAGS  efMotionEventsUnprocessedThisPass;		// used for debugging purposes ONLY, AND of efMotionEventsUponEntry and efMotionEvents[eMotor] at exit from User FSM

	if(eMotor IS MOTOR_NONE)
		return;


	efMotionEventsUponEntry = efMotionEvents[eMotor];			// keep a copy of Motion Events upon entry to FSM

    // ************************************************************************
    //							State Transition
    // ************************************************************************
    // state transitions are based on:
    //
	//	    external events
    //		state complete
    //
    // state transitions handle setting up everything BEFORE entering the new state
    // if there is no need for a transition, we just stay in the current state

	// external events may indicate multiple, contradictory state transitions.
	// the order of processing of external events determines their absolute priority; the FIRST successfully processed transition will actually occur

    switch(eMotionState[eMotor])
    {

		case ST_MOTION_INIT:
			fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// initialize substate variables before changing states
			bMotionFSMSequenceCtr[eMotor] = 0;

            eMotionState[eMotor] = ST_MOTION_STOPPED;						// force state change
	        break;

		// *************************************************
		//				STOPPED
		// *************************************************
        case ST_MOTION_STOPPED:				// initial state

			// End-of-Travel detection overrides any other state
			// (this may appear redundant, but has the effect of resetting the state)
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// this is the primary EXIT Event from this state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_ACCELERATE;				// clears EF_MOTION_ACCELERATE_CMD upon entry
				}

	        break;


		// *************************************************
		//				ACCELERATE
		// *************************************************
        case ST_MOTION_ACCELERATE:			// accelerate by increasing PWM

			// End-of-Travel detection overrides any other state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;				// clears EF_MOTION_STALLED upon entry
				break;														// no possible alternative action; exit state now
				}

			// check substate sequence counter to make sure we have processed enough of the state to test for exit transitions...
			if (bMotionFSMSequenceCtr[eMotor] < 1)
				{
				// we have not processed enough of the state to exit yet..
				break;
				}

			// this is one of THREE primary EXIT events for this state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
				{
				// user has requested STOP, so sequence to decelerate
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_DECELERATE;						// clears EF_MOTION_DECELERATE_CMD upon entry
				break;														// no possible alternative action; exit state now
				}

			// this is one of THREE primary EXIT events for this state
			// NOTE: if we are doing a SHORT MOVE, rather than a MOVE, the User FSM will have already adjusted the pgwStateMSILimit
			// we always want to run out the (adjusted) MSI tick count, and transition on EF_MOTION_MSI_COUNT_DONE
			if ((pgwStateMSICtr >= pgwStateMSILimit) OR (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE)))
				{
				// we have completed the required number of Motion Sensor ticks during acceleration
				// If we are overcurrent, too, it will be picked up in the constant speed state
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				// ==> this transition really should be to CONSTANT_SPEED OR DECELERATE, depending on whether or not there is a RUN phase

				eMotionState[eMotor] = ST_MOTION_CONSTANT_SPEED;					// clears EF_MOTION_MSI_COUNT_DONE on entry
				break;														// no possible alternative action; exit state now
				}

			// this is one of THREE primary EXIT events for this state
			if ((fSubStateStatus[eMotor] IS SUBSTATE_DONE) OR (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM)))
				{
				// we have run out of PWM steps OR we have increased to the maximum required PWM
				// Note that we are assuming that this is NOT possible during a SHORT MOVE
				// If we are overcurrent, too, it will be picked up in the Constant Speed state
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				// ==> this transition really should be to CONSTANT_SPEED OR DECELERATE, depending on whether or not there is a RUN phase

				eMotionState[eMotor] = ST_MOTION_CONSTANT_SPEED;					// clears EF_MOTION_MAXIMUM_PWM on entry
				break;														// no possible alternative action; exit state now
				}

			// *********************************
			// Motion Control Adjustment Transitions
			// *********************************
			// Motion Control adjustement event
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW))
				{
				// Motion Sensor tick too long, increase PWM

				// if the previous PWM adjustment put us at the maximum PWM, we CANNOT increase the PWM
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
					{
					// no need for a runtime error/event log, this is to be expected
					// clear the flag, we cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);		// clear event flag; cannot act on it

					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);			// clear event flag; cannot act on it
					}
				else
					{
					// change state to increase PWM
					// this is a single pass state, so there is no need to set fSubStateStatus[eMotor] or bMotionFSMSequenceCtr
					eMotionState[eMotor] = ST_MOTION_ACC_INCREASE_PWM;				// clears EF_MOTION_TOO_SLOW upon entry
					}
				break;
				}

			// Motion Control adjustement event
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST))
				{
				// Motion Sensor tick too short; reduce PWM

				// if the previous PWM adjustment put us at the maximum PWM, we are clearly NOT adjusting in the correct direction
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
					{
					// clear the flag, we cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);
					RuntimeError(MTN_FSM_ERROR_TOO_FAST_MAX_PWM);

					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);			// clear event flag; we cannot act on it
					// no change of state
					}
					// if the previous PWM adjustment put us at the minimum PWM, we CANNOT decrease the PWM
				else if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM))
					{
					// clear the flag, we cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);
					RuntimeError(MTN_FSM_ERROR_BELOW_MINIMUM_PWM);

					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);			// clear event flag; we cannot act on it
					// no change of state
					}
				else
					{
					// change state to decrease PWM
					eMotionState[eMotor] = ST_MOTION_ACC_DECREASE_PWM;				// clears EF_MOTION_TOO_FAST, EF_MOTION_MINIMUM_PWM upon entry
					}
				break;
				}

			// check for overcurrent, change state to ST_MOTION_ACC_DECREASE_PWM, would go here

	        break;		// end of case ST_MOTION_ACCELERATE

		// *********************************
		// (Acceleration) Speed Adjustment States
		// *********************************
		case ST_MOTION_ACC_INCREASE_PWM:	// increase PWM to adjust speed
			// this is inplemented as a single pass state, so the default transition is to return to ST_MOTION_ACCELERATE,

			#ifdef ALLOW_ACC_STALL
				// Motion STALLED overrides any other state change
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
					{
					fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
					bMotionFSMSequenceCtr[eMotor] =0;
	
		            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;						// clears EF_MOTION_STALLED upon entry
					break;														// no possible alternative action; exit state now
					}
			#endif 	// ALLOW_ACC_STALL

			// no other action possible
            eMotionState[eMotor] = ST_MOTION_ACCELERATE;
	        break;

        case ST_MOTION_ACC_DECREASE_PWM:	// decrease PWM to prevent overcurrent or adjust speed
			// this is inplemented as a single pass state, so the default transition is to return to ST_MOTION_ACCELERATE
            eMotionState[eMotor] = ST_MOTION_ACCELERATE;

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;				// clears EF_MOTION_STALLED upon entry
				break;														// no possible alternative action; exit state now
				}

			// check for overcurrent, change state to ST_MOTION_ACC_DECREASE_PWM, would go here
			// check for OK current, change state to ST_MOTION_ACCELERATE, would go here
	        break;


		// *************************************************
		//				CONSTANT SPEED 'RUN'
		// *************************************************
        case ST_MOTION_CONSTANT_SPEED:		// constant speed, constant PWM

			// End-of-Travel detection overrides any other state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;				// clears EF_MOTION_STALLED upon entry
				break;														// no possible alternative action; exit state now
				}

			// check substate sequence counter to make sure we have processed enough of the state to test for exit transitions...
			if (bMotionFSMSequenceCtr[eMotor] < 1)
				{
				// we have not processed enough of the state to exit yet..
				break;
				}

			#ifdef USE_INCLINOMETER_FEEDBACK
				// With Inclinometer feedback, the Constant Speed phase is complete when we reach the destination angle.
				// Current, averaged angle is tested against destination angle in Inclinometer.c:MotionSensor_Tick()
				// ==>> distance traveled during deceleration and coasting WILL be a motion error
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_ANGLE_MOVE_DONE))
					{
					pgMotionStats[eMotor].efConstantSpeedMotionEvents |= efMotionEvents[eMotor];	// keep a copy of the event flag, will not be stored by this state
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_ANGLE_MOVE_DONE);					// clear calling flag
					BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_ANGLE_MOVE_DONE);			// keep a copy of the ending command

					fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
					bMotionFSMSequenceCtr[eMotor] = 0;

					eMotionState[eMotor] = ST_MOTION_DECELERATE;				// clears EF_MOTION_DECELERATE_CMD upon entry
					break;														// no possible alternative action; exit state now
					}
			#endif


			// this is one of TWO primary EXIT events for this state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
				{
				// user has requested STOP, so sequence to decelerate
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_DECELERATE;				// clears EF_MOTION_DECELERATE_CMD upon entry
				break;														// no possible alternative action; exit state now
				}

			// this is one of TWO primary EXIT events for this state
			// ==> this needs to account for a possibly shortened accleration phase...
			// ==> this is the exit path for a SHORT MOVE (pgwStateMSICtr == pgwStateMSILimit == 0), which does not actually have a (meaningful) constant speed RUN phase
			if ((pgwStateMSICtr >= pgwStateMSILimit) OR (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE)))
				{
				// we have completed the required number of quadrature ticks during constant speed
				// If we are overcurrent, too, it will be picked up in the deceleration state
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_DECELERATE;						// clears EF_MOTION_MSI_COUNT_DONE on entry
				break;														// no possible alternative action; exit state now
				}


			// *********************************
			// Motion Control Adjustment Transitions
			// *********************************
			// Motion Control adjustment event
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW))
				{
				// Motion Sensor tick too long, increase PWM

				// if the previous PWM adjustment put us at the maximum PWM, we CANNOT increase the PWM
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
					{
					// no need for a runtime error/event log, this is to be expected
					// clear the flag, we cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);		// clear event flag; cannot act on it

					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);			// clear event flag; cannot act on it
					// no change of state
					}
				else
					{
					// change state to increase PWM
					// this is a single pass state, so there is no need to set fSubStateStatus[eMotor] or bMotionFSMSequenceCtr
					eMotionState[eMotor] = ST_MOTION_CON_INCREASE_PWM;				// logs and clears EF_MOTION_TOO_SLOW flag upon entry
					}
				break;														// no possible alternative action; exit state now
				}

			// Motion Control adjustment event
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST))
				{
				// Motion Sensor tick too short; reduce PWM

				// if the previous PWM adjustment put us at the maximum PWM, we are clearly NOT adjusting in the correct direction
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
					{
					// clear the flag, we cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);
					RuntimeError(MTN_FSM_ERROR_TOO_FAST_MAX_PWM);

					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);			// clear event flag; we are clearly confused
					}
					// if the previous PWM adjustment put us at the minimum PWM, we CANNOT decrease the PWM
				else if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM))
					{
					// clear the flag, we cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);
					RuntimeError(MTN_FSM_ERROR_BELOW_MINIMUM_PWM);

					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);			// clear event flag; we cannot act on it
					// no change of state
					}
				else
					{
					// change state to decrease PWM
					// this is {PRESENTLY} a single pass state, so there is no need to set fSubStateStatus[eMotor] or bMotionFSMSequenceCtr
					eMotionState[eMotor] = ST_MOTION_CON_DECREASE_PWM;				// logs and clears EF_MOTION_TOO_FAST flag upon entry
					}
				break;														// no possible alternative action; exit state now
				}

			// check for overcurrent, change state to ST_MOTION_CON_DECREASE_PWM, would go here

			// if the previous PWM adjustment put us at the maximum PWM and the CORRECT speed, the above will NOT have cleared the event!
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
				{
				// no need for a runtime error/event log, this is to be expected
				// clear the flag, we cannot act on it
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);		// clear event flag; cannot act on it
				RuntimeError(MTN_FSM_ERROR_MAXIMUM_PWM);
				}

	        break;

		// *********************************
		//	Speed Adjustment States
		// *********************************
		case ST_MOTION_CON_INCREASE_PWM:	// increase PWM to adjust speed
			// this is inplemented as a single pass state

			#ifdef ALLOW_ACC_STALL
				// I think this state is an attempt to recover from a stall during motion.. 
				// Motion STALLED overrides any other state change
				if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
					{
					fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
					bMotionFSMSequenceCtr[eMotor] = 0;
	
		            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;						// clears EF_MOTION_STALLED upon entry
					break;														// no possible alternative action; exit state now
					}
			#endif		// ALLOW_ACC_STALL

			// no other action possible
            eMotionState[eMotor] = ST_MOTION_CONSTANT_SPEED;
	        break;

        case ST_MOTION_CON_DECREASE_PWM:	// decrease PWM to prevent overcurrent
			// this is inplemented as a single pass state, so the default transition is to return to ST_MOTION_CONSTANT_SPEED

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;				// clears EF_MOTION_STALLED upon entry
				break;														// no possible alternative action; exit state now
				}

			// check for overcurrent, change state to ST_MOTION_CON_DECREASE_PWM, would go here
			// check for OK current, change state to ST_MOTION_CONSTANT_SPEED, would go here

			// no other action possible
            eMotionState[eMotor] = ST_MOTION_CONSTANT_SPEED;
	        break;


		// *************************************************
		//				DECELERATE
		// *************************************************
        case ST_MOTION_DECELERATE:			// decelerate by reducing PWM

			// End-of-Travel detection overrides any other state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;				// clears EF_MOTION_STALLED upon entry
				break;														// no possible alternative action; exit state now
				}

			// check substate sequence counter to make sure we have processed enough of the state to test for exit transitions...
			if (bMotionFSMSequenceCtr[eMotor] < 1)
				{
				// we have not processed enough of the state to exit yet..
				break;
				}

			// this is one of THREE primary EXIT events for this state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM))
				{
				// we are at the minimum PWM value, so we will have to continue at constant PWM
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_MINIMUM_PWM;
				break;														// no possible alternative action; exit state now
				}

			// this is one of THREE primary EXIT events for this state
			if (fSubStateStatus[eMotor] IS SUBSTATE_DONE)
				{
				// we have run out of PWM steps
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_MINIMUM_PWM;
				break;														// no possible alternative action; exit state now
				}

			// this is one of THREE primary EXIT events for this state
			// if we have not transitioned out of this state as a result of running out of PWM steps, 
			// we should run out the (adjusted) MSI tick count, and transition on EF_MOTION_MSI_COUNT_DONE
			if ( /*(pgwStateMSICtr >= pgwStateMSILimit) OR */ (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE)))
				{
				// we have completed the required number of quadrature ticks during deceleration
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_COASTING;							// allow system to coast to a halt; clears EF_MOTION_MSI_COUNT_DONE on entry
				break;														// no possible alternative action; exit state now
				}

			// *********************************
			// Motion Control Adjustment Transitions
			// *********************************
			// these possible input events don't do anything specific, because we are already decelerating
			// Motion Control adjustement event
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST))
				{
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);

				// Motion Sensor tick too short; reduce PWM
//				eMotionState[eMotor] = ST_MOTION_DEC_DECREASE_PWM;
				}

			// check for overcurrent, and MAYBE change state to ST_MOTION_DEC_DECREASE_PWM, would go here

			// Motion Control adjustement event
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW))
				{
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);

				// Motion Sensor tick too long, increase PWM
//				eMotionState[eMotor] = ST_MOTION_DEC_INCREASE_PWM;
				}

	        break;


		// *************************************************
		//			CONSTANT SPEED, MINIMUM PWM
		// *************************************************
        case ST_MOTION_MINIMUM_PWM:			// constant speed at minimum PMW

			// End-of-Travel detection overrides any other state
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_SOFT_STALL;				// clears EF_MOTION_STALLED upon entry
				break;														// no possible alternative action; exit state now
				}

			// this is the ONLY EXIT event for this state
			// we always want to run out the (adjusted) MSI tick count, and transition on EF_MOTION_MSI_COUNT_DONE
			if ( /*(pgwStateMSICtr >= pgwStateMSILimit) OR */ (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE)))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_COASTING;					// allow system to coast to a halt; clears EF_MOTION_MSI_COUNT_DONE on entry
				break;														// no possible alternative action; exit state now
				}

			// is this even necessary? isn't the primary issue the likelyhood of a STALL?
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW))
				{
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
				// Motion Sensor tick too long, increase PWM
//				eMotionState[eMotor] = ST_MOTION_DEC_INCREASE_PWM;
				}

	        break;

		// *************************************************
		//				COASTING TO A HALT
		// *************************************************
        case ST_MOTION_COASTING:			// coasting to a halt.. may only be relevant for testing partial hardware

			// For this ONE state, Motion STALLED means we are finally done with motion
			// substate sequence (handled within the state)
			//		wait 25mS
			//		apply brake
			//		wait 25mS
			//		check for MOTION_STALL

			// when ALL of the above has completed, fSubStateStatus[eMotor] is set to SUBSTATE_DONE

			if (fSubStateStatus[eMotor] IS SUBSTATE_DONE)
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_STOPPED;					// next state is STOPPED
				}
			break;

		// *************************************************
		//			SOFT STALL (recovery)
		// *************************************************
        case ST_MOTION_SOFT_STALL:				// stall recovery state

			// if we have gotten multiple MSI ticks, we have recovered from the stall, and can transition to coasting
			// in a stall condition, we want to allow either event to cause a transition
			if ((pgwStateMSICtr >= pgwStateMSILimit) OR (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE)))
				{
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

				eMotionState[eMotor] = ST_MOTION_COASTING;						// allow system to coast to a halt
				break;													// no possible alternative action; exit state now
				}

			// if we have run through the entire set of substates without a MSI tick, we are done with stall recovery
			if (fSubStateStatus[eMotor] IS SUBSTATE_DONE)
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] = 0;

	            eMotionState[eMotor] = ST_MOTION_HARD_STALL;					// next state is HARD STALL, recovery has failed
				break;														// no possible alternative action; exit state now
				}


			// if we get a motion stall timeout, we are done with stall recovery
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
				{
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);			// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionFSMSequenceCtr[eMotor] =0;

	            eMotionState[eMotor] = ST_MOTION_HARD_STALL;					// next state is HARD STALL, recovery has failed
				}
	        break;


		// *************************************************
		//				HARD STALL (NO EXIT!)
		// *************************************************
        case ST_MOTION_HARD_STALL:				// stalled, error state
			// no transition out of STALLED; requires power OFF or reset
	        break;

	    default:
            RuntimeError(MTN_FSM_ERROR_INVALID_STATE);
            break;

    }	// end of state transtion



    // *************************************************
    //		    Process the Current or New State
    // *************************************************
    // State handler
    //	called one or more times for a state, depending on state implementation
    //	state transitions do NOT occur here
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate


    switch(eMotionState[eMotor])
    {

		case ST_MOTION_INIT:
	        RuntimeError(MTN_FSM_ERROR_INVALID_STATE);
            break;

		// *************************************************
		//					STOPPED
		// *************************************************
        case ST_MOTION_STOPPED:				// initial state
			// turn OFF all PWM drivers

			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					DisableMotorDrive();					// disable BOTH MC99326 Motor Drive ICs

					StopMotionStallCtr(eMotor);				// disable any subsequent Motion Stalled timeouts, sets eMotionStallMotor to MOTOR_NONE

					// ******************** DISABLE MSI INTERRUPTS *********************
					// first entry into state, disable subsequent MSI interrupts so that we do not get spurious interrupts in the STOPPED state
					MotionSensor_DisableInt(eMotor);		// if we are using Inclinometer feedback, this will stop scheduled reading of the Accelerometer

					// first entry into state, so tell MotionPhase FSM we ARE STOPPED
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED);

					// if we were sent here by EF_MOTION_END_OF_TRAVEL, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL);
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_END_OF_TRAVEL);	// keep a copy of the ending command

						// normalize the position counter to the end of travel (==> is this correct relative to Limit switches? Is there comparable forward code?)
						if (eMotor IS MOTOR_AZIMUTH)
							CurrentPosition_Set(eMotor, AZ_ALLOWABLE_POS_REVERSE);
						else if (eMotor IS MOTOR_ELEVATION)
							CurrentPosition_Set(eMotor, EL_ALLOWABLE_POS_REVERSE);

////						SetLEDs(LEDS_END_OF_TRAVEL);
						}

					// if the PWM duty cycle is not 0 when we enter this state, something is wrong..
					if (fgbAdjustedDutyCycle[eMotor] IS_NOT PWM_DUTY_CYCLE_OFF)
						{
						RuntimeError(MTN_FSM_ERROR_DUTY_CYCLE_NOT_MIN);
						}

					// finish collecting the motion stats
					Finish_MotionStats(eMotor);

					// reset all speed calculations
					CurrentSpeed_Write(eMotor, (MOTION_PROFILE_SPEED_TYPE)ZERO);
					
					// if we are not already stopped, set PWM to stopped 
					if (pgePWMConfig[eMotor] IS_NOT PWM_CONFIG_STOPPED)		// <sek> 20 Sep 13
						{
						// make sure the Motor Drive Bridge is OFF, sets pgeMotionType = MOTION_COASTING
						PWM_SetConfig(eMotor, PWM_CONFIG_STOPPED);		// PWM drive is stopped, does NOT mean Movement has stopped
						}

					pgeMotionType[eMotor] = MOTION_STOPPED;				// override motion type, motion is STOPPED

					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_STOPPED;

					// stop the PWM
					fgbAdjustedDutyCycle[eMotor] = PWM_DUTY_CYCLE_OFF;	// set PWM value to 0
					pgcDutyCycleCorrection[eMotor] = 0;					// set PWM correction value to 0
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, 0);	// stop the PWM hardware counters

					// clear any and ALL events that may be left from a previous move (EF_MOTION_MSI_COUNT_DONE)
					efMotionEvents[eMotor] = 0;							// clear ALL motion event flags

					// all motion must be complete, so clear the bIsMotionStarted flag JUST ONCE, upon entry into this state
					// (set and queried externally from this FSM by MenuFSM, MoveSequenceFSM)
					ClearMotionStarted(eMotor);

					++bMotionFSMSequenceCtr[eMotor];					// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state
					// check for EF_RESULT_STOPPED command overrun
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))
						{
						RuntimeError(MTN_FSM_ERROR_STOP_OVERRUN);
						// note that we do not want to actually clear the flag - it may take the User FSM longer to respond..
						}
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;

		// *************************************************
		//					ACCELERATE
		// *************************************************
        case ST_MOTION_ACCELERATE:					// accelerate by increasing PWM

			// track ALL events observed during state
			pgMotionStats[eMotor].efAccelerationMotionEvents |= efMotionEvents[eMotor];

			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					// **first entry** into acceleration state
					// processed just ONCE per move
					EnableMotorDrive();						// enable MC33926 motor driver ICs (disabled during ST_MOTION_STOPPED)

					// capture START of timing information 
					pgMotionStats[eMotor].lStartTime = (UINT32)ReadMSITimer(eMotor);

					#ifdef MOTION_ERROR_TABLE
						// do NOT clear the Motion Error table if we are in stall recovery, we would like to see the stall occur!
						if (eMoveType[eMotor] IS_NOT MOVE_STALL_RECOVERY)
						{
							// clear the motion error table (used for debugging only at this time..)
							ClearMotionErrorTable();
						}
					#endif

					// if we were sent here by the MotionPhaseFSM, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD))
						{
						BITSET(pgMotionStats[eMotor].efStartingEvent, EF_MOTION_ACCELERATE_CMD);	// keep a copy of the starting command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD);					// clear calling flag
						}


					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_ACCELERATION;

					// first entry into state, so initialize quadrature tick counter
					pgwStateMSICtr[eMotor] = 0;								// restart state encoder tick counter
					pgwStateMSILimit[eMotor] = pgwMSIAccelerationCount[eMotor];		// Motion Sensor ticks required for acceleration

					#ifdef USE_HALL_SENSOR_FEEDBACK
						// start the motion stall counter, which would otherwise not start until the first MSI tick
						// (if we wait for the first MSI tick, we would be dependent on motion to detect the LACK of motion!)
						// update stall counter to 1.5 times (?) longest expected interval
						SetMotionStallCtr(eMotor, MTN_SENSOR_STARTUP_STALL_PERIOD(eMotor));
					#endif

					// ******************** ENABLE MSI INTERRUPTS *********************
					// first entry into state, enable MSI interrupts for motion stall detection  (this is the ONLY place this is done..)
					MotionSensor_EnableInt(eMotor);													// ==>> duplicate of call in PWM_SetConfig()?

					// initialize motion profile table indicies
					fgbMotionProfilePWMIndex[eMotor] = pgbMotionProfileIndexMin[eMotor];			// initialize motion profile index for reading PWM values
					pgbMotionProfileSpeedIndex[eMotor] = pgbMotionProfileIndexMin[eMotor];			// initialize motion profile index for reading MSI Tick Speed values (used in MotionSensor.c)

					// initialize PWM
					fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];		// initialize PWM
					pgMotionStats[eMotor].bPWM_AccelDutyCycleMin = fgbAdjustedDutyCycle[eMotor];			// keep track of minimum PWM
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);

					// if we are using the feedback simulator (no motor or Motion Sensors attached), set the initial simulation timer value
					#ifdef USE_FEEDBACK_SIMULATOR
						MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);		// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
					#endif

					// clear the EF_MOTION_MINIMUM_PWM flag, which we really do not care about right now..
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);

					// keep track of motion profile index used, this will be overwritten in the next substate
					pgMotionStats[eMotor].bPWMAccelMotionIndex = fgbMotionProfilePWMIndex[eMotor];

					// capture end-of-state timing information, this will be overwritten in the next substate
					pgMotionStats[eMotor].lAccelerationEndTime = (UINT32)ReadMSITimer(eMotor);

					++bMotionFSMSequenceCtr[eMotor];								// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state (including the first MSI tick, which CANNOT generate a TOO FAST/SLOW flag)
					// should be caused by EF_MOTION_PWM_UPDATE
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);

						// calculate new motion profile index, and look up new duty cycle 
						if (fgbMotionProfilePWMIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
							{
							// bump motion profile table index
							fgbMotionProfilePWMIndex[eMotor] += pgbMotionProfileIndexIncrement[eMotor];

							// read new PWM from motion profile table
							fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

							// NOTE: this has the potential for creating a PWM value > 100%, which will be limit checked by PWM_SetDutyCycle()
							// add in motion control PWM correction (if any) ==> this needs to be bounds checked!
							fgbAdjustedDutyCycle[eMotor] += pgcDutyCycleCorrection[eMotor];

							// keep track of maximum duty cycle during acceleration
							if (fgbAdjustedDutyCycle[eMotor] > pgMotionStats[eMotor].bPWM_AccelDutyCycleMax)
								{
								pgMotionStats[eMotor].bPWM_AccelDutyCycleMax = fgbAdjustedDutyCycle[eMotor];
								}

							// update PWM duty cycle value
							IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);

							// keep track of the number of actual adjustments
							++pgMotionStats[eMotor].wAccelerationPWMAdjustmentCount;
							}
						else
							{
							// exit from state is caused by external event (EF_MOTION_MAXIMUM_PWM or (pgwStateMSICtr >= pgwStateMSILimit)), so there is nothing else to do here
							++bMotionFSMSequenceCtr[eMotor];		// force next substate on next FSM entry
							fSubStateStatus[eMotor] = SUBSTATE_DONE;
							}

						// capture end-of-state timing information (updated on each pass through this state until we exit)
						pgMotionStats[eMotor].lAccelerationEndTime = (UINT32)ReadMSITimer(eMotor);
						}
					else if (efMotionEvents[eMotor] IS 0)
						{
						// we must be here as a result of a timer event, and we don't need to do anything with it..
						;
						}
					else
						{
						////RuntimeError(MTN_FSM_ERROR_NON_PWM_UPDATE_TICK);
						// keep a copy of the events that put us here..
						fgefMotionEventsCopy[eMotor] |= efMotionEvents[eMotor];				// error 551: (Warning -- Symbol 'fgefMotionEventsCopy' not accessed)
						}

					// keep track of motion profile index used
					pgMotionStats[eMotor].bPWMAccelMotionIndex = fgbMotionProfilePWMIndex[eMotor];

					break;

				case 2:
					RuntimeError(MTN_FSM_ERROR_TOO_MANY_TICKS);
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;

		// *********************************
		// Motion Control Adjustment States
		// *********************************
        case ST_MOTION_ACC_INCREASE_PWM:			// increase PWM to adjust speed

			// ==> this is called as a result of a MSI tick, so we DO need to bump the motion profile index

			// track ALL events observed during state
			pgMotionStats[eMotor].efAccelerationMotionEvents |= efMotionEvents[eMotor];

			// if we were sent here by the the EF_MOTION_TOO_SLOW flag, clear the event flag
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW))
				{
				// BITSET(pgMotionStats[eMotor].efStartingEvent, EF_MOTION_TOO_SLOW);		// keep a copy of the starting command
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);

				// calculate new motion profile index, and look up new duty cycle
				// motion profile index is not bounded by pre-calculated maximum motion profile index for acceleration
				if (fgbMotionProfilePWMIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
					{
					// bump motion profile table index
					fgbMotionProfilePWMIndex[eMotor] += pgbMotionProfileIndexIncrement[eMotor];

					// read current PWM from motion profile table
					fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

					// if current PWM + previous correction value < maximum PWM, we can bump the correction value again
					if ((fgbAdjustedDutyCycle[eMotor] + pgcDutyCycleCorrection[eMotor]) < PWM_DUTY_CYCLE_MAX(eMotor))
					{
						// OK to bump correction value
						// we are here to adjust the PWM, so adjust the Duty Cycle correction value
						++pgcDutyCycleCorrection[eMotor];

						// keep track of the number of actual adjustments
						++pgMotionStats[eMotor].wAccelerationPWMAdjustmentCount;

						// keep track of maximum correction value
						if (pgcDutyCycleCorrection[eMotor] > pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMax)
							{
							pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMax = pgcDutyCycleCorrection[eMotor];
							}
					}

					// NOTE: if we are already at the maximum possible PWM correction, pgcDutyCycleCorrection[eMotor] will be the previous value, so we STILL need to add it
					// add in motion control PWM correction (if any)
					// despite PC-Lint complaints, this is correct. fgbAdjustedDutyCycle is unsigned, and pgcDutyCycleCorrection is signed
					fgbAdjustedDutyCycle[eMotor] += pgcDutyCycleCorrection[eMotor];

					// bounds check the resulting duty cycle value <sek> 4 Apr 13
					if (fgbAdjustedDutyCycle[eMotor] > PWM_DUTY_CYCLE_MAX(eMotor))		// <sek> 16 Aug 13 was compared to 100, could compare to pgbPWMDutyCycleMax[eMotor]
						{
						RuntimeError(MTN_FSM_ERROR_ABOVE_MAXIMUM_PWM);
						// limit the adjusted PWM value
						fgbAdjustedDutyCycle[eMotor] = PWM_DUTY_CYCLE_MAX(eMotor);
						}

					// keep track of maximum duty cycle during acceleration
					if (fgbAdjustedDutyCycle[eMotor] > pgMotionStats[eMotor].bPWM_AccelDutyCycleMax)
						{
						pgMotionStats[eMotor].bPWM_AccelDutyCycleMax = fgbAdjustedDutyCycle[eMotor];
						}

					// update PWM duty cycle value
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);
					}
				}
	        break;

        case ST_MOTION_ACC_DECREASE_PWM:			// decrease PWM to prevent overcurrent

			// ==> this is called as a result of a MSI tick, so even though we are trying to slow down, we DO need to bump the motion profile index

			// track ALL events observed during state
			pgMotionStats[eMotor].efAccelerationMotionEvents |= efMotionEvents[eMotor];

			// if we were sent here by the the EF_MOTION_TOO_FAST flag, clear the event flag
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST))
				{
				// BITSET(pgMotionStats[eMotor].efStartingEvent, EF_MOTION_TOO_FAST);		// keep a copy of the starting command
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);

				// calculate new motion profile index, and look up new duty cycle 
				if (fgbMotionProfilePWMIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
					{
					// bump motion profile table index
					fgbMotionProfilePWMIndex[eMotor] += pgbMotionProfileIndexIncrement[eMotor];

					// read current PWM from motion profile table
					fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

					// we are here to adjust the PWM, so adjust the Duty Cycle correction value
					// bounds check to make sure we do not calculate a value that may result in motion stopping
					// despite PC-Lint complaints, this is correct. fgbAdjustedDutyCycle is unsigned, and pgcDutyCycleCorrection is signed
					if ((fgbAdjustedDutyCycle[eMotor] + pgcDutyCycleCorrection[eMotor] - 1) > PWM_DUTY_CYCLE_MIN)		// pgbPWMDutyCycleMin[eMotor]
					{
						--pgcDutyCycleCorrection[eMotor];

						// keep track of the number of actual adjustments
						++pgMotionStats[eMotor].wAccelerationPWMAdjustmentCount;

						// keep track of minimum correction value
						if (pgcDutyCycleCorrection[eMotor] < pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMin)
							{
							pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMin = pgcDutyCycleCorrection[eMotor];
							}
					}

					// add in motion control PWM correction (if any, note that correction may be negative!)
					fgbAdjustedDutyCycle[eMotor] += pgcDutyCycleCorrection[eMotor];

					// bounds check the resulting duty cycle value <<== is this necessary?
					if (fgbAdjustedDutyCycle[eMotor] <= pgbPWMDutyCycleMin[eMotor])
						{
						RuntimeError(MTN_FSM_ERROR_BELOW_MINIMUM_PWM);

						fgbAdjustedDutyCycle[eMotor] = pgbPWMDutyCycleMin[eMotor];
						}

					// no need to keep track of maximum duty cycle here; cannot be a new maximum!
					// keep track of minimum duty cycle during acceleration
					if (fgbAdjustedDutyCycle[eMotor] < pgMotionStats[eMotor].bPWM_AccelDutyCycleMin)
						{
						pgMotionStats[eMotor].bPWM_AccelDutyCycleMin = fgbAdjustedDutyCycle[eMotor];
						}

					// update PWM duty cycle value
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);
					}
		        break;
				}

			// check for EF_MOTION_HIGH_CURRENT flag would go here...
	        break;

		// *************************************************
		//				CONSTANT SPEED
		// *************************************************
        case ST_MOTION_CONSTANT_SPEED:		// constant speed, constant PWM

			// track ALL events observed during state
			pgMotionStats[eMotor].efConstantSpeedMotionEvents |= efMotionEvents[eMotor];

			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					// **first entry** into constant speed state
					// processed just ONCE per move
					// if we were sent here (from ST_MOTION_ACCELERATE) by the the EF_MOTION_MAXIMUM_PWM flag, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_MAXIMUM_PWM);		// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);
						}

					// we transitioned here by the EF_MOTION_MSI_COUNT_DONE flag, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_MSI_COUNT_DONE);	// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);				// clear event flag
						}

					// set flag to tell MotionPhaseFSM that acceleration is completed
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE);

					// clear any dangling events from PHASE_ACCELERATION
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);

					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_CONSTANT_SPEED;

					// initialize Run phase duty cycle limits
					pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMax = fgbAdjustedDutyCycle[eMotor];
					pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMin = fgbAdjustedDutyCycle[eMotor];
					pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMax = pgcDutyCycleCorrection[eMotor];				// initialize to correction value determined during acceleration
					pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMin = pgcDutyCycleCorrection[eMotor];				// initialize to correction value determined during acceleration

					// first entry into state, so initialize Motion Sensor tick counter
					// if the system was SLOW during Acceleration, the PWM adjustment may run into the maximum PWM value before running out of pre-calculated MSI ticks
					//		check for acceleration completed MSI ticks
					if (pgwStateMSICtr[eMotor] < pgwMSIAccelerationCount[eMotor])			// <sek> 16 Aug 13
					{
						// Acceleration did not complete required ticks, so add missing ticks to constant speed MSI tick limit
						pgwStateMSILimit[eMotor] = (pgwMSIAccelerationCount[eMotor] - pgwStateMSICtr[eMotor]);
					}
					else
					{
						// accleration completed, so initialize constant speed MSI tick Limit to 0
						pgwStateMSILimit[eMotor] = 0;
					}

					pgwStateMSICtr[eMotor] = 0;										// restart state encoder tick counter
					pgwStateMSILimit[eMotor] += pgulMSIConstantSpeedCount[eMotor];	// add maximum Motion Sensor ticks at constant speed 'run'  ==>> this needs to be UINT32
					++bMotionFSMSequenceCtr[eMotor];								// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent state entries
					// check for EF_RESULT_ACCELERATION_DONE overrun
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE))
						{
						RuntimeError(MTN_FSM_ERROR_ACCELERATION_DONE_OVERRUN);
						// note that we do not want to actually clear the flag - it may take the User FSM longer to respond
						}

					// capture end-of-state timing information (updated on each pass through this state)
					pgMotionStats[eMotor].lConstantSpeedEndTime = (UINT32)ReadMSITimer(eMotor);
					break;

				case 2:
					RuntimeError(MTN_FSM_ERROR_UNEXPECTED_EVENT);
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;

		// *********************************
		// Motion Control Adjustment States
		// *********************************
		case ST_MOTION_CON_INCREASE_PWM:	// accelerate PWM to overcome load and maintain constant speed

			// track ALL events observed during state
			pgMotionStats[eMotor].efConstantSpeedMotionEvents |= efMotionEvents[eMotor];

			// if we were sent here by the the EF_MOTION_TOO_SLOW flag, clear the event flag
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW))
				{
				BITSET(pgMotionStats[eMotor].efStartingEvent, EF_MOTION_TOO_SLOW);		// keep a copy of the starting command
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
				}

			// bounds check the motion profile index to make sure we do not go off the end of the Motion Profile table
			if (fgbMotionProfilePWMIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
				{
				// read current PWM from motion profile table
				fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

				// if current PWM + previous correction value < maximum PWM, we can bump the correction value again
				if ((fgbAdjustedDutyCycle[eMotor] + pgcDutyCycleCorrection[eMotor]) < PWM_DUTY_CYCLE_MAX(eMotor))
				{
					// OK to bump correction value
					// we are here to adjust the PWM, so adjust the Duty Cycle correction value
					++pgcDutyCycleCorrection[eMotor];

					// keep track of the number of actual adjustments
					++pgMotionStats[eMotor].ulConstantSpeedPWMAdjustmentCount;

					// keep track of maximum correction value
					if (pgcDutyCycleCorrection[eMotor] > pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMax)
						{
						pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMax = pgcDutyCycleCorrection[eMotor];
						}
				}
	
				// add in motion control PWM correction 
				fgbAdjustedDutyCycle[eMotor] += pgcDutyCycleCorrection[eMotor];

				// bounds check (should not be necessary)

				// keep track of maximum duty cycle 
				if (fgbAdjustedDutyCycle[eMotor] > pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMax)
					{
					pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMax = fgbAdjustedDutyCycle[eMotor];
					}

				// update PWM duty cycle value
				IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);
				}

			// this is a single pass state, so the next time the FSM is called, we will move out of this state
			break;

        case ST_MOTION_CON_DECREASE_PWM:	// decrease PWM to prevent overcurrent

			// track ALL events observed during state
			pgMotionStats[eMotor].efConstantSpeedMotionEvents |= efMotionEvents[eMotor];

			// if we were sent here by the the EF_MOTION_TOO_FAST flag, clear the event flag
			if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST))
				{
				BITSET(pgMotionStats[eMotor].efStartingEvent, EF_MOTION_TOO_FAST);		// keep a copy of the starting command
				BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);
				}

			// bounds check the motion profile index (not really necessary..)
			if (fgbMotionProfilePWMIndex[eMotor] >= pgbMotionProfileIndexMin[eMotor])
				{
				// read new PWM from motion profile table
				fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

				// we are here to adjust the PWM, so adjust the Duty Cycle correction value
				// bounds check to make sure we do not calculate a duty cycle value that would cause motion to stop
				if ((fgbAdjustedDutyCycle[eMotor] + pgcDutyCycleCorrection[eMotor] - 1) > PWM_DUTY_CYCLE_MIN)		// pgbPWMDutyCycleMin[eMotor]
				{
					--pgcDutyCycleCorrection[eMotor];

					// keep track of the number of actual adjustments
					++pgMotionStats[eMotor].ulConstantSpeedPWMAdjustmentCount;

					// keep track of minimum correction value
					if (pgcDutyCycleCorrection[eMotor] < pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMin)
						{
						pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMin = pgcDutyCycleCorrection[eMotor];
						}
				}

				// add in motion control PWM correction (if any, may be a negative correction)
				// despite PC-Lint complaints, this is correct. fgbAdjustedDutyCycle is unsigned, and pgcDutyCycleCorrection is signed
				fgbAdjustedDutyCycle[eMotor] += pgcDutyCycleCorrection[eMotor];

				// bounds check the resulting duty cycle value <<== is this necessary?
				if (fgbAdjustedDutyCycle[eMotor] <= pgbPWMDutyCycleMin[eMotor])
					{
					// force minimum PWM value
					fgbAdjustedDutyCycle[eMotor] = pgbPWMDutyCycleMin[eMotor];
					RuntimeError(MTN_FSM_ERROR_BELOW_MINIMUM_PWM);
					}

				// keep track of minimum duty cycle 
				if (fgbAdjustedDutyCycle[eMotor] < pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMin)
					{
					pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMin = fgbAdjustedDutyCycle[eMotor];
					}

				// update PWM duty cycle value
				IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);
				}

			// this is a single pass state, so the next time the FSM is called, we will move out of this state
	        break;

		// *************************************************
		//					DECELERATE
		// *************************************************
        case ST_MOTION_DECELERATE:			// decelerate by reducing PWM

			// track ALL events observed during state
			pgMotionStats[eMotor].efDecelerationMotionEvents |= efMotionEvents[eMotor];

			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					// if we were sent here by the MotionPhaseFSM, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_DECELERATE_CMD);		// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD);
						}
					else
						{
						// send a flag to the User FSM to tell it we are done with constant speed 'run' phase
						BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE);
						}

					// if the previous PWM adjustment put us at the maximum PWM, clear the event flag and discard it..
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
						{
						// no need for a runtime error/event log, this is to be expected
						// clear the flag, we cannot act on it
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);
						}

					// we transitioned here by the EF_MOTION_MSI_COUNT_DONE flag, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_MSI_COUNT_DONE);	// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);				// clear event flag
						}

					// update Motion Phase; affects Motion Sensor  Encoder tick counting
					pgeMotionPhase[eMotor] = PHASE_DECELERATION;

					// first entry into state, so initialize Motion Sensor  tick counter
					pgwStateMSICtr[eMotor] = 0;													// restart state encoder tick counter
					pgwStateMSILimit[eMotor] = pgwMSIDecelerationCount[eMotor];					// Motion Sensor ticks for deceleration, MAY include Minimum PWM ticks
					++bMotionFSMSequenceCtr[eMotor];											// force next substate on next FSM entry

					// keep track of maximum duty cycle during deceleration
					pgMotionStats[eMotor].bPWM_DecelDutyCycleMax = fgbAdjustedDutyCycle[eMotor];	// max duty cycle is the Constant Speed duty cycle
					pgMotionStats[eMotor].bPWMDecelMotionIndex = fgbMotionProfilePWMIndex[eMotor];	// initial Motion Profile Index is the Constant Speed motion profile index

					// clear any 'dangling' unprocessed motion control events from previous (Constant Speed) state
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);						// clear event flag; cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);						// clear event flag; cannot act on it

					break;
					// lint -fallthrough

				case 1:
					// all subsequent entries into the state
					// should be caused by EF_MOTION_PWM_UPDATE (set by MotionSensor.c: MotionSensor_Tick()
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);

						// look up new duty cycle
						// Bounds Check new Motion Profile index
						// make sure that bumping the profile index will still leave us with a valid profile table index
						if (fgbMotionProfilePWMIndex[eMotor] >= (pgbMotionProfileIndexMin[eMotor] + pgbMotionProfileIndexDecrement[eMotor]))
							{
							// bump motion profile table index (note that for deceleration we decrement the index, moving towards stopped)
							fgbMotionProfilePWMIndex[eMotor] -= pgbMotionProfileIndexDecrement[eMotor];

							// 10 Apr 13 keep track of MotionProfile table index for Speed Error table
							pgbMotionProfileSpeedIndex[eMotor] = fgbMotionProfilePWMIndex[eMotor];

							// read new PWM from motion profile table
							fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

							// bounds check to make sure we do not calculate a duty cycle value that would cause motion to stop
							if ((fgbAdjustedDutyCycle[eMotor] + pgcDutyCycleCorrection[eMotor] - 1) > PWM_DUTY_CYCLE_MIN)		// pgbPWMDutyCycleMin[eMotor]
							{
								// add in motion control PWM correction (if any, may be a negative correction)
								// despite PC-Lint complaints, this is correct. fgbAdjustedDutyCycle is unsigned, and pgcDutyCycleCorrection is signed
								fgbAdjustedDutyCycle[eMotor] += pgcDutyCycleCorrection[eMotor];
							}
							else
							{
								// force minimum PWM value
								fgbAdjustedDutyCycle[eMotor] = pgbPWMDutyCycleMin[eMotor];
								RuntimeError(MTN_FSM_ERROR_BELOW_MINIMUM_PWM);
							}

							// keep track of minimum duty cycle during deceleration
							if (fgbAdjustedDutyCycle[eMotor] < pgMotionStats[eMotor].bPWM_DecelDutyCycleMin)
								{
								pgMotionStats[eMotor].bPWM_DecelDutyCycleMin = fgbAdjustedDutyCycle[eMotor];
								}

							// update PWM duty cycle value
							IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);
							}
						else if (fgbMotionProfilePWMIndex[eMotor] > pgbMotionProfileIndexMin[eMotor])			// corrected, was missing indicies without compiler error! <sek> 10 Apr 13
							{
							// we are too close to the minimum index value to bump the profile table index again!
							// just use the minimum valid index value
							fgbMotionProfilePWMIndex[eMotor] = pgbMotionProfileIndexMin[eMotor];

							// 10 Apr 13 keep track of MotionProfile table index for Speed Error table
							pgbMotionProfileSpeedIndex[eMotor] = fgbMotionProfilePWMIndex[eMotor];

							// read new PWM from motion profile table, no need for adjustment; this will be the minimum allowable value
							fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

							// keep track of minimum duty cycle during deceleration
							pgMotionStats[eMotor].bPWM_DecelDutyCycleMin = fgbAdjustedDutyCycle[eMotor];

							// update PWM duty cycle value
							IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);
							}
						else
							{
							// exit from state is caused by external event (EF_MOTION_MINIMUM_PWM), so there is nothing else to do here
							++bMotionFSMSequenceCtr[eMotor];		// force next substate on next FSM entry
							fSubStateStatus[eMotor] = SUBSTATE_DONE;
							}

						// if the previous PWM adjustment put us at the maximum PWM, something is wrong..
						if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
							{
							// no need for a runtime error/event log, this is to be expected
							// clear the flag, we cannot act on it
							BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);
							RuntimeError(MTN_FSM_ERROR_MAXIMUM_PWM);
							}

						// if we got a DECELERATE_CMD after completing the RUN phase by counting MSI ticks, something is wrong..
						if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
							{
							BITCLEAR(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD);
							RuntimeError(MTN_FSM_ERROR_MAXIMUM_PWM);
							}

						// capture end-of-state timing information (updated on each pass through this state)
						pgMotionStats[eMotor].lDecelerationEndTime = (UINT32)ReadMSITimer(eMotor);

						}
					else
						{
						////RuntimeError(MTN_FSM_ERROR_NON_PWM_UPDATE_TICK);
						// keep a copy of the events that put us here..
						fgefMotionEventsCopy[eMotor] |= efMotionEvents[eMotor];		// error 551: (Warning -- Symbol 'fgefMotionEventsCopy' not accessed)
						}

					// check for overrun - see if the User FSM has cleared EF_RESULT_CONSTANT_SPEED_DONE
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
						{
						// NOT the right error message, it should do for now
						RuntimeError(MTN_FSM_ERROR_ACCELERATION_DONE_OVERRUN);
						}
					break;

				case 2:
					RuntimeError(MTN_FSM_ERROR_TOO_MANY_TICKS);
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;

		// *************************************************
		//			CONSTANT SPEED, MINIMUM PWM
		// *************************************************
        case ST_MOTION_MINIMUM_PWM:			// constant speed at minimum PMW

			// track ALL events observed during state
			pgMotionStats[eMotor].efMinimumPWMMotionEvents |= efMotionEvents[eMotor];

			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					// if we were sent here by hitting MINIMUM_PWM, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_MINIMUM_PWM);		// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);
						}

					// we should NOT transition here by the EF_MOTION_MSI_COUNT_DONE flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE))
						{
						RuntimeError(MTN_FSM_ERROR_UNEXPECTED_EVENT);					// flag error
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);				// clear event flag
						}

					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_MINIMUM_PWM;

					// clear any 'dangling' unprocessed EF_MOTION_PWM_UPDATE events from ST_MOTION_DECELERATE
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);

					// note that we do NOT set a new value for pgwStateMSILimit, because this state exists only to complete deceleration MSI ticks
					++bMotionFSMSequenceCtr[eMotor];							// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// capture timing information (updated on each pass through this state)
					pgMotionStats[eMotor].lMinimumPWMEndTime = (UINT32)ReadMSITimer(eMotor);

					// no need to set fSubStateStatus[eMotor] or bump bMotionFSMSequenceCtr
					// exit from state is caused by an external event - pgwStateMSICtr done or EF_MOTION_MSI_COUNT_DONE
					break;

				case 2:
					RuntimeError(MTN_FSM_ERROR_UNEXPECTED_EVENT);
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;

		// *************************************************
		//				COASTING TO A HALT
		// *************************************************
		// Note: the (expected) EF_MOTION_STALL event may occur at any time during this state. It must be processed immediately, 
		// because leaving it set until the end of the state will result in continuous re-exection of the Motion FSM - and it will hog the MCU

        case ST_MOTION_COASTING:			// coasting to a halt.. may only be relevant for testing partial hardware

			// track ALL events observed during state
			pgMotionStats[eMotor].efCoastingMotionEvents |= efMotionEvents[eMotor];

			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					// clear flag that will be used to capture EF_MOTION_STALL event
					bStallHasOccured[eMotor] = FALSE;

					// if we transitioned here by the EF_MOTION_MSI_COUNT_DONE flag, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_MSI_COUNT_DONE);	// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);				// clear event flag
						}

					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_COASTING;

					// clear any 'dangling' unprocessed EF_MOTION_PWM_UPDATE events from ST_MOTION_DECELERATE
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);

					// if the PWM duty cycle is not at the minimum (index) value when we enter this state, something is wrong..
					if (fgbMotionProfilePWMIndex[eMotor] > pgbMotionProfileIndexMin[eMotor])
						{
						RuntimeError(MTN_FSM_ERROR_DUTY_CYCLE_NOT_MIN);
						}

					// first entry into state, turn off the PWM drivers
					// make sure the Motor Drive Bridge is OFF, sets pgeMotionType = MOTION_COASTING
					PWM_SetConfig(eMotor, PWM_CONFIG_STOPPED);								// sets MOTION_COASTING

					// stop the PWM
					fgbAdjustedDutyCycle[eMotor] = PWM_DUTY_CYCLE_OFF;						// set PWM value to 0
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, 0);						// stop the PWM hardware counters

					// start the Motion 25mS Tick timer
					BITSET(efMotionTimerEvents[eMotor], EF_MTN_TIMER);

					// check for stall
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);				// clear event flag
						bStallHasOccured[eMotor] = TRUE;									// capture event
						}
					++bMotionFSMSequenceCtr[eMotor];				// force next substate on next FSM entry
					break;

				// adding a 25mS delay here does NOT seem to improve stopping performance (number of coasting ticks), but it probably IS a good idea
				case 1:
					// check for stall
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);				// clear event flag
						bStallHasOccured[eMotor] = TRUE;									// capture event
						}

					// one FSM call delay (25mS) between turning off the PWM and applying the brake
					// turn on the electric brake
					if (IS_BITSET(efMotionTimerEvents[eMotor], EF_MTN_TIMER_25MS_TICK))
						{
						BITCLEAR(efMotionTimerEvents[eMotor], EF_MTN_TIMER_25MS_TICK);		// clear event flag

						PWM_SetConfig(eMotor, PWM_CONFIG_BRAKE);							// sets MOTION_BRAKING

						++bMotionFSMSequenceCtr[eMotor];			// force next substate on next FSM entry
						}

						// ==> add RuntimeError() call for unexpected tick?
					break;

				case 2:
					// check for stall (this is acceptable and expected here, it is the indication that coasting has stopped..)
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);				// clear event flag
						bStallHasOccured[eMotor] = TRUE;									// capture event
						}

					// 2nd 25mS Delay
					if (IS_BITSET(efMotionTimerEvents[eMotor], EF_MTN_TIMER_25MS_TICK))
						{
						BITCLEAR(efMotionTimerEvents[eMotor], EF_MTN_TIMER_25MS_TICK);		// clear event flag

						// end the Motion 25mS Tick timer
						BITCLEAR(efMotionTimerEvents[eMotor], EF_MTN_TIMER);

						++bMotionFSMSequenceCtr[eMotor];			// force next substate on next FSM entry
						}
					break;

				case 3:
					// all subsequent entries into the state
					// (we MAY get here before a motion stall occurs, or the motion stall may occur before we get here..)
					// if a motion stall has occured, we are done with the entire coast/brake/stall sequence
					//if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED) OR (bStallHasOccured[eMotor] IS_TRUE))			// <sek> 17 Aug 13
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);				// clear event flag
						bStallHasOccured[eMotor] = FALSE;									// clear captured event flag

						fSubStateStatus[eMotor] = SUBSTATE_DONE;							// done with entire substate sequence
						}
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}

			// capture end-of-state timing information (updated on each pass through this state)
			pgMotionStats[eMotor].lCoastEndTime = (UINT32)ReadMSITimer(eMotor);

			break;


		// *************************************************
		//				SOFT STALL (recovery)
		// *************************************************
		// this state actually includes the entire Stall FSM as substates

        case ST_MOTION_SOFT_STALL:				// stall recovery state
			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:				// STOP all current motion
					DisableMotorDrive();					// disable BOTH MC99326 Motor Drive ICs

					// disable any subsequent Motion Stalled timeouts
					StopMotionStallCtr(eMotor);				// disable any subsequent Motion Stalled timeouts, sets eMotionStallMotor to MOTOR_NONE

					// ******************** DISABLE MSI INTERRUPTS *********************
					// first entry into state, disable subsequent MSI interrupts so that we do not get spurious interrupts in the STOPPED state
////					MotionSensor_DisableInt(eMotor);


					// finish collecting the motion stats BEFORE changing anything
					Finish_MotionStats(eMotor);

					// NOTE: compared to the normal ST_MOTION_STOPPED, we do NOT call
					// DisableMotorDrive()								// no need to disable driver, we will restart shortly
					// MotionSensor_DisableInt()


					// make sure the Motor Drive Bridge is OFF, sets pgeMotionType = MOTION_COASTING
					PWM_SetConfig(eMotor, PWM_CONFIG_STOPPED);			// calls PWM_SetDutyCycle(eMotor, PWM_DUTY_CYCLE_OFF)
																		// does not update pgePWMDirection because PWM_DIR_STOPPED is NOT a direction
					pgeMotionType[eMotor] = MOTION_STOPPED;				// override motion type, motion is STOPPED

					// if we were sent here by a motion stall or high current, clear the event flag
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_STALLED))
						{
						BITSET(pgMotionStats[eMotor].efEndingEvent, EF_MOTION_STALLED);			// keep a copy of the ending command
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);
						}

					// check for EF_MOTION_HIGH_CURRENT would go here

					if (efMotionEvents[eMotor] IS_NOT 0)
						{
						// there must be some other event; if there is any OTHER way for us to get here, I don't know what it is!
						RuntimeError(MTN_FSM_ERROR_UNEXPECTED_EVENT);
						// clear ALL other pending motion events. We are in a STALL, the only thing that matters is recovery!
						efMotionEvents[eMotor] = 0;
						}

					// tell MotionPhaseFSM we are in a soft stall state
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL);

					// update Motion Phase; affects Motion Sensor tick counting. We are NOT really stopped yet!
					pgeMotionPhase[eMotor] = PHASE_COASTING;

					// reset all speed calculations
					CurrentSpeed_Write(eMotor, (MOTION_PROFILE_SPEED_TYPE)ZERO);

					// stop the PWM
					fgbAdjustedDutyCycle[eMotor] = 0;							// set PWM value to 0

					// waiting until the next substate acts a simple delay of one Motion FSM execution tick - 25mS
					// start the Motion 25mS Tick timer
					BITSET(efMotionTimerEvents[eMotor], EF_MTN_TIMER);

					fgbSoftStallDelayCtr = 0;							// start count of 25mS ticks for delay

					++bMotionFSMSequenceCtr[eMotor];					// force next substate on next FSM entry
					break;


				case 1:				// delay before restart, then setup stall recovery
					// this was originally just one FSM call delay (25mS) between turning off the PWM and reversing direction
					// the delay was extended to allow for manual testing (using metal tweezers to short out the Motion Sensor ticks from the Hall Sensor)
					if (IS_BITSET(efMotionTimerEvents[eMotor], EF_MTN_TIMER_25MS_TICK))
						{
						BITCLEAR(efMotionTimerEvents[eMotor], EF_MTN_TIMER_25MS_TICK);		// clear event flag
						if (++fgbSoftStallDelayCtr IS SOFT_STALL_RECOVERY_DELAY_COUNT)		// delay done?
							{
							BITCLEAR(efMotionTimerEvents[eMotor], EF_MTN_TIMER);			// turn OFF motion timer
							}
						else
							{
							break;				// delay not done, so done with this pass through substate
							}
						}
					else
						{
						// for some reason we are here without having timed out..
						// wait for NEXT call into FSM
						RuntimeError(MTN_FSM_ERROR_UNEXPECTED_TICK);		// <<= we are getting unexpected ticks here
						break;
						}

					// the intent here is NOT to have any event flags set, so we get a normal execution delay of 25mS
					if (efMotionEvents[eMotor] IS_NOT 0)
						{
						RuntimeError(MTN_FSM_ERROR_UNEXPECTED_EVENT);
						}

					// perhaps make this another substate?
					// this looks a lot like ST_MOTION_ACCELERATE, because we are starting a new move - but not a normal move!

					EnableMotorDrive();						// enable MC33926 motor driver ICs (disabled during ST_MOTION_STOPPED)

					// first entry into state, so clear the timers (reduced possibility of overflow)
					ClearMSITimer(eMotor);

					// set new move limits - we are just setting the number of MSI ticks in each part of the motion
					eMoveType[eMotor] = MOVE_STALL_RECOVERY;								// Stall Recovery move
					pgMotionStats[eMotor].eMoveType = MOVE_STALL_RECOVERY;
					IGNORE_RETURN_VALUE SetMotionLimits(eMotor, eMoveType[eMotor]);			// set MSI tick counts for each part of motion - nothing conditional about this..
																							// initializes pgwStateMSICtr, pgwStateMSILimit

					// set new PWM bridge direction, based on SetMotionLimits()
					// this appears largely redundant to SetMotionLimits(), but it DOES allow us to set the direction LEDs
					if (pgeStallRecoveryPWMConfig[eMotor] IS PWM_CONFIG_FORWARD)
						{
						PWM_SetConfig(eMotor, PWM_CONFIG_FORWARD);			// pgeMotionType = MOTION_STARTING

						// set LED display states - for all other conditions this is handled by the User FSM
////						SetLEDs(LEDS_FORWARD);								// set direction LEDs
						}
					else if (pgeStallRecoveryPWMConfig[eMotor] IS PWM_CONFIG_REVERSE)
						{
						PWM_SetConfig(eMotor, PWM_CONFIG_REVERSE);			// pgeMotionType = MOTION_STARTING

						// set LED display states - for all other conditions this is handled by the User FSM
////						SetLEDs(LEDS_REVERSE);								// set direction LEDs
						}
					else
						{
						// not a valid direction
						RuntimeError(MTN_FSM_ERROR_INVALID_DIRECTION);
						}

					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_ACCELERATION;

					// start the motion stall counter, which would otherwise not start until the first MSI tick
					// (if we wait for the first MSI tick, we would be dependent on motion to detect the LACK of motion!)
					// update stall counter to 3 times longest expected interval
					SetMotionStallCtr(eMotor, MTN_SENSOR_RECOVERY_STALL_PERIOD(eMotor));

					// capture START of timing information 
					pgMotionStats[eMotor].lStartTime = (UINT32)ReadMSITimer(eMotor);


					//==>> we need to call SetMotionLimits() BY THIS POINT, see ABOVE
					// initialize motion profile table indicies
					fgbMotionProfilePWMIndex[eMotor] = pgbMotionProfileIndexMin[eMotor];		// initialize motion profile index for reading PWM values, as initialized by SetMotionLimits()
					// redundant to SetMotionLimits() pgbMotionProfileSpeedIndex[eMotor] = pgbMotionProfileIndexMin[eMotor];		// initialize motion profile index for reading MSI Tick Speed values

					// initialize PWM to minimum value from MotionProfile table
					fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];		// initialize PWM
					pgMotionStats[eMotor].bPWM_AccelDutyCycleMin = fgbAdjustedDutyCycle[eMotor];		// keep track of minimum PWM
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);			// start the PWM

					// capture end-of-state timing information (updated on each pass through this state until we exit)
					pgMotionStats[eMotor].lAccelerationEndTime = (UINT32)ReadMSITimer(eMotor);

					++bMotionFSMSequenceCtr[eMotor];											// force next substate on next FSM entry
					break;

				case 2:
					// acceleration entries into the state - ramping up the PWM

					// entry should be caused by EF_MOTION_PWM_UPDATE OR 25mS Motion Timer tick
					// unlike acceleration above, we want to process BOTH event types
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
						}

					// this is essentially a duplicate of code found in MotionSensor.c: MotionSensorInterruptHandler(),
					// but it must execute when the system is STALLED and thus NOT generating MotionSensor interrupts
					// It operates on 25mS MotionTimer ticks, instead
					// calculate new motion profile index, and look up new duty cycle 
					if (fgbMotionProfilePWMIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
						{
						// bump motion profile table index
						fgbMotionProfilePWMIndex[eMotor] += pgbMotionProfileIndexIncrement[eMotor];

						// read new PWM from motion profile table
						fgbAdjustedDutyCycle[eMotor] = (BYTE)MotionProfileSpeedAndPWM[eMotor][(fgbMotionProfilePWMIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET];

						// keep track of maximum duty cycle during acceleration
						pgMotionStats[eMotor].bPWM_AccelDutyCycleMax = fgbAdjustedDutyCycle[eMotor];

						// update PWM duty cycle value
						IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, fgbAdjustedDutyCycle[eMotor]);

						// if the previous PWM adjustment put us at the maximum PWM, we just want to discard the flag..
						if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM))
							{
							// clear the flag, we cannot act on it - we are waiting for either Motion Stall Timeout or MSI Ticks..
							BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);	// clear flag; we cannot act on it
							RuntimeError(MTN_FSM_ERROR_MAXIMUM_PWM);
							}
						}
					else
						{
						// exit from state is caused by external event (pgwStateMSICtr >= pgwStateMSILimit), so there is nothing else to do here
						++bMotionFSMSequenceCtr[eMotor];		// force next substate on next FSM entry
//						fSubStateStatus[eMotor] = SUBSTATE_DONE;
						}

					// clear events that will be ignored during stall recovery
					// - I think these are overkill - TOO_SLOW and MAX_PWM observed, and make sense for a stall!
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);

					// clear any 'dangling' unprocessed motion control events
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);				// clear event flag; cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);				// clear event flag; cannot act on it

					// capture end-of-state timing information (updated on each pass through this state until we exit)
					pgMotionStats[eMotor].lAccelerationEndTime = (UINT32)ReadMSITimer(eMotor);
					break;

				case 3:
					// all subsequent entries into the state
					// should be caused by EF_MOTION_PWM_UPDATE OR 25mS tick
					// unlike acceleration above, we want to process BOTH event types
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE))
						{
						BITCLEAR(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
						}

					// we are now applying full PWM in an attempt to get the motor to move!

					// count timer ticks to limit the amount of time we apply full PWM
					++pgulMSI_ConstantSpeedCtr[eMotor];

					if (pgulMSI_ConstantSpeedCtr[eMotor] >= pgulMSIConstantSpeedCount[eMotor])
						{
						// exit from state is caused by external event (pgwStateMSICtr >= pgwStateMSILimit), so there is nothing else to do here
						++bMotionFSMSequenceCtr[eMotor];		// force next substate on next FSM entry
						fSubStateStatus[eMotor] = SUBSTATE_DONE;
						}

					// - I think these are overkill - TOO_SLOW and MAX_PWM observed, and make sense for a stall!
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);

					// clear any 'dangling' unprocessed motion control events
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);				// clear event flag; cannot act on it
					BITCLEAR(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);				// clear event flag; cannot act on it

					// capture end-of-state timing information (updated on each pass through this state until we exit)
					pgMotionStats[eMotor].lConstantSpeedEndTime = (UINT32)ReadMSITimer(eMotor);

					break;

				case 4:
					RuntimeError(MTN_FSM_ERROR_TOO_MANY_TICKS);
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;


		// *************************************************
		//				HARD STALL (NO EXIT!)
		// *************************************************
        case ST_MOTION_HARD_STALL:				// stalled, error state
			switch(bMotionFSMSequenceCtr[eMotor])
				{
				case 0:
					// tell User FSM we are in a hard stall state
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL);

					// update Motion Phase; affects Motion Sensor tick counting
					pgeMotionPhase[eMotor] = PHASE_STOPPED;

					// disable any subsequent Motion Stalled timeouts
					StopMotionStallCtr(eMotor);							// disable any subsequent Motion Stalled timeouts, sets eMotionStallMotor to MOTOR_NONE

					// if the PWM duty cycle is not at the minimum value when we enter this state, something is wrong..
					if (fgbAdjustedDutyCycle[eMotor] > pgbPWMDutyCycleMin[eMotor])
						{
						RuntimeError(MTN_FSM_ERROR_DUTY_CYCLE_NOT_MIN);
						}

					// finish collecting the motion stats
					Finish_MotionStats(eMotor);

					// reset all speed calculations
					CurrentSpeed_Write(eMotor, (MOTION_PROFILE_SPEED_TYPE)ZERO);

					// make sure the Motor Drive Bridge is OFF, sets pgeMotionType = MOTION_COASTING
					PWM_SetConfig(eMotor, PWM_CONFIG_STOPPED);
					pgeMotionType[eMotor] = MOTION_STOPPED;				// override motion type, motion is STOPPED

					// stop the PWM
					fgbAdjustedDutyCycle[eMotor] = 0;					// set PWM value to 0
					IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, 0);	// stop the PWM hardware counters

					++bMotionFSMSequenceCtr[eMotor];					// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state
					// ==>we do NOT need to check for EF_RESULT_HARD_STALL overrun, because it is a hard error - we do not exit
					break;

				default:
					RuntimeError(MTN_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}

	        break;


	    default:
            RuntimeError(MTN_FSM_ERROR_INVALID_STATE);
            break;

    }	// end of state processing


	// *************************************************
	//		check for any unprocessed events
	// *************************************************
	// we will consider an event unprocessed
	// Note: this means that an asynchronous external event that was generated during THIS pass through the User FSM
	//       will NOT be consider unprocessed during THIS execution pass

	// AND Motion Events upon FSM entry with Motion Events NOW to determine what was NOT processed during this pass
	// note the we are masking out the TOO_FAST and TOO_SLOW events, which may take more than one pass to clear
	efMotionEventsUnprocessedThisPass = efMotionEventsUponEntry & efMotionEvents[eMotor] & EF_MOTION_MULTI_PASS_EVENTS_MASK;	// flags that were set upon entry and are STILL set now will be a 1

	if (efMotionEventsUnprocessedThisPass IS_NOT 0)						// any unprocessed events?
		{
		RuntimeError(MTN_FSM_ERROR_UNPROCESSED_EVENT);					// flag runtime error
//		efUnprocessedMotionEvents |= efMotionEvents[eMotor];					// keep a log of unprocessed events
		efUnprocessedMotionEvents[eMotor] |= efMotionEventsUnprocessedThisPass;	// keep a log of unprocessed events
		}

#ifdef UNPROCESSED_EVENT_ERROR
	// some events may remain unprocessed for a few passes.. we want to track only those times when there are unprocessed events

	if (efMotionEvents[eMotor] IS_NOT 0)										// any unprocessed events?
		{
		RuntimeError(MTN_FSM_ERROR_UNPROCESSED_EVENT);					// flag runtime error
		}
#endif
}

// end of MotionFSM.c

