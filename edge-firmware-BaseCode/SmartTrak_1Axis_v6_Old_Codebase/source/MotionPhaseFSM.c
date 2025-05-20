// *************************************************************************************************
//									M o t i o n P h a s e F S M . c
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Phase FSM, started by motion Commands
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

// Nomenclature:

// RIGHT/UP = Forward
// LEFT/DOWN = Reverse


enum tagMotionPhaseFSMErrors
{
	MTN_PHASE_FSM_ERROR_NONE = MTN_PHASE_FSM_ERROR_BASE,
	MTN_PHASE_FSM_ERROR_UNEXPECTED_TICK,		// 1 unexpected timer tick event
	MTN_PHASE_FSM_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	MTN_PHASE_FSM_ERROR_INVALID_STATE,			// 3 not a valid state
	MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	MTN_PHASE_FSM_ERROR_INVALID_MOVE,			// 5 not a valid move type (see enums below)
	MTN_PHASE_FSM_ERROR_SOFT_STALL,				// 6 stall recovery
	MTN_PHASE_FSM_ERROR_HARD_STALL,				// 7 motion stall error
	MTN_PHASE_FSM_ERROR_STOP_CMD_OVERRUN,		// 8 command not processed
	MTN_PHASE_FSM_ERROR_ACCEL_CMD_OVERRUN,		// 9 command not processed
	MTN_PHASE_FSM_ERROR_DECEL_CMD_OVERRUN,		// A command not processed
	MTN_PHASE_FSM_ERROR_CANNOT_FIND_CENTER,		// B incorrect location during Find Center
	MTN_PHASE_FSM_ERROR_NOT_AT_CENTER,			// C incorrect location during Find Center
												// D
												// E
	MTN_PHASE_FSM_ERROR_UNPROCESSED_EVENT = MTN_PHASE_FSM_ERROR_BASE + 0x0F
};


//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

PRIVATE_INIT BYTE bMotionPhaseFSMSequenceCtr[NUM_MOTORS] = {0, 0};								// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus[NUM_MOTORS] = {SUBSTATE_NOT_DONE, SUBSTATE_NOT_DONE};		// substate status flag

//lint -esym(552,efUnprocessedUserEvents)		error 552: (Warning -- Symbol 'efUnprocessedUserEvents' not accessed)
/*PRIVATE_INIT*/ EVENTFLAGS  efUnprocessedUserEvents[NUM_MOTORS] = {0, 0};					// used for debugging purposes ONLY, to track unprocessed events

//-------------------------------------------------------------------------------------------------------
// Static Function Prototypes
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Function Bodies
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
//  MotionFSM
//
//  Input: 	    none
//  Output:	    none
//  Desc:		motion control FSM
//-------------------------------------------------------------------------------------------------------

// Command FSM states
// These states describe the interaction between the MotionFSM and the MotionPhaseFSM
enum tagMotionPhaseStates
{
    ST_MTN_PHASE_FSM_INIT,				// initial state, only at power up

    ST_MTN_PHASE_FSM_STOPPED,			// all motion stopped

    ST_MTN_PHASE_FSM_ACCELERATE_FWD,	// accelerate forward (right, up)
    ST_MTN_PHASE_FSM_CONSTANT_SP_FWD,	// constant speed forward
	ST_MTN_PHASE_FSM_DECELERATE_FWD,	// decelerate foward to stop

    ST_MTN_PHASE_FSM_ACCELERATE_REV,	// accelerate reverse (left, down)
    ST_MTN_PHASE_FSM_CONSTANT_SP_REV,	// move to right
	ST_MTN_PHASE_FSM_DECELERATE_REV,	// decelerate left to stop

	ST_MTN_PHASE_FSM_SOFT_STALL,		// stall recovery
	ST_MTN_PHASE_FSM_HARD_STALL			// stalled error state, no recovery possible

};

enum tagMotionPhaseStates eMotionPhaseState[NUM_MOTORS] = {ST_MTN_PHASE_FSM_INIT, ST_MTN_PHASE_FSM_INIT};			// User FSM state variable
enum tagMoveTypes eLastMoveType[NUM_MOTORS] = {MOVE_NONE, MOVE_NONE};					// used to track moves in FindEndPoints sequence


// this is an array of pointers to state descriptor strings
// NOTE: this array must exactly track the above enum in order for it to make any sense!
FILE_GLOBAL ARRAY const char *pstrMotionPhaseStateText[] =
	{
	"Init",
	"Stopped",
	"Acc FWD (Rt/Up)",
	"Move FWD (Rt/Up)",
	"Dec FWD (Rt/Up)",
	"Acc REV (Lt/Dn)",
	"Move REV (Lt/Dn)",
	"Dec REV (Lt/Dn)",
	"Find End FWD (Rt/Up)",
	"Find End REV (Lt/Dn)",
	"Soft Stall",
	"Hard Stall",
	""
	};


const char *GetMotionPhaseStateString(enum tagMotors eMotor)
{
	// return point to state descriptor string
	// note that we are not doing any bounds checking..
	return (pstrMotionPhaseStateText[eMotionPhaseState[eMotor]]);

}



// *************************************************************************************************
//								S y n c h r o n i z a t i o n 
// *************************************************************************************************

// these functions exist to avoid the problem of synchronizing external events with move commands.

// When a process external to the MotionPhaseFSM (specifically serial Menu Commands and the Move Sequencer) starts a Command,
// it also needs to know when the Command is complete. The state of efMotionPhaseCommands is NOT an adequate indicator because
// the initiating flag is CLEARED as soon as the command STARTS, and gives no indication of when it actually completes.
// Overall system timing (see the dispatcher in Main.c) will typically allow the external process to query
// efMotionPhaseCommands before the Command has even started to do anyting - and interpret it as Command complete.

// so the idea here is that the external process (specifically serial Menu Commands and the Move Sequencer) calls SetCommandStarted()
// to indicate that a move is in process, setting the fgbIsMotionStarted flag to TRUE.

// When the MotionPhaseFSM enters ST_MTN_PHASE_FSM_STOPPED from some other state, the FIRST substate of ST_MTN_PHASE_FSM_STOPPED calls
// ClearCommandStarted() JUST ONCE to clear the fgbIsMotionStarted flag to FALSE.

FILE_GLOBAL_INIT BOOL fgbIsCommandStarted[NUM_MOTORS]  = {FALSE, FALSE};

void SetCommandStarted(enum tagMotors eMotor)
	{
	fgbIsCommandStarted[eMotor] = TRUE;
	}

void ClearCommandStarted(enum tagMotors eMotor)
	{
	fgbIsCommandStarted[eMotor] = FALSE;
	}

BOOL IsCommandComplete(enum tagMotors eMotor)
	{
	if (fgbIsCommandStarted[eMotor])
		// Command has started, so it cannot be complete
		return FALSE;
	else
		return TRUE;
	}


// *************************************************************************************************
//								C o m m a n d F S M ( )
// *************************************************************************************************


/*
	FSM Events
		New Motion Command
		Overcurrent
		Overcurrent Stalled
*/

// this Finite State Machine is called repeatedly on timer ticks and input events
// execution is flow-through-and-exit; we never stay in this routine
// delays are handled by external timers and subsequent calls to this routine

void MotionPhaseFSM(enum tagMotors eMotor)
{

	EVENTFLAGS  efMotionResultEventsUponEntry;				// used for debugging purposes ONLY, keeps a copy of User Event flags at entry to User FSM
	EVENTFLAGS  efMotionResultEventsUnprocessedThisPass;	// used for debugging purposes ONLY, AND of efMotionResultEventsUponEntry and efMotionResultEvents[eMotor] at exit from User FSM

	if(eMotor IS MOTOR_NONE)
		return;

	efMotionResultEventsUponEntry = efMotionResultEvents[eMotor];	// keep a copy of User Events upon entry to FSM

    // ************************************************************************
    //						State Transition
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

    switch(eMotionPhaseState[eMotor])
    {

		case ST_MTN_PHASE_FSM_INIT:					// initial state, only at power up
			fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// initialize substate variables before changing states
			bMotionPhaseFSMSequenceCtr[eMotor] = 0;

            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;			// force state change
	        break;


		// *************************************************
		//					STOPPED
		// *************************************************
        case ST_MTN_PHASE_FSM_STOPPED:				// all motion stopped

			// we are STOPPED, so we can process a new move and state change

			// *****************************
			//			STOP
			// *****************************

			// Any reason to stop overrides any other state change
			// (this may appear redundant, but has the effect of resetting the state)
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;														// no possible alternative action; reset state now
				}

			// *****************************
			//			STALL
			// *****************************
			// STALL handling
			// NOTE: this is the only place that we can transition to SOFT_STALL or HARD_STALL
			// all other states use EF_RESULT_SOFT_STALL, EF_RESULT_HARD_STALL to transfer to here

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL))
				{
				// do not clear the event flag, it will be cleared in ST_MTN_PHASE_FSM_SOFT_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_SOFT_STALL;
				break;														// no possible alternative action; exit state now
				}

			// Stall recovery failure overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL))
				{
				// do not clear the event flag, it will be cleared in ST_MTN_PHASE_FSM_HARD_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_HARD_STALL;
				break;														// no possible alternative action; exit state now
				}


			// *****************************
			//	Check for State Complete
			// *****************************
			if (fSubStateStatus[eMotor] IS_NOT SUBSTATE_DONE)
				// ST_MTN_PHASE_FSM_STOPPED state processing is not complete, so we cannot even consider processing a commanded state transition
				{
				break;
				}


			// *******************************************
			//			Process Command Event Flags
			// *******************************************
			// command events may be initiated by the serial menu OR MoveSequenceFSM

			// *****************************
			//  Start Finite Distance Move
			// *****************************
			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE FWD command, start a move
			// Forward = RIGHT/UP, positive position numbers

			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_RUN_FWD))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_RUN_FWD);	// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_RUN_FWD;							// RUN Forward, maximum speed at up to maximum allowable distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				if (SetMotionLimits(eMotor, eMoveType[eMotor]) IS_TRUE)		// calculate and set motion phase limits and total motion limits
					{
					eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_FWD; // start acceleration Forward, will test for end limit; substate variables initialized above
					}
				else
					{
					eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;	// motion limits problem; cannot move, so force state change; substate variables initialized above
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED);// to force behavior as if TOLD to stop
					}
				break;														// done with move setup; exit state now
				}

			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE REVERSE command, start a move
			// REVERSE == LEFT/DOWN, Negative position numbers

			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_RUN_REV))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_RUN_REV);	// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_RUN_REV;							// RUN Reverse, maximum speed at up to maximum allowable distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				if (SetMotionLimits(eMotor, eMoveType[eMotor]) IS_TRUE)		// calculate and set motion phase limits and total motion limits
					{
					eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;	// start acceleration Reverse, will test for reverse limit
					}
				else
					{
					eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;	// motion limits problem; cannot move, so force state change; substate variables initialized above
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED); // to force behavior as if TOLD to stop
					}
				break;														// done with move setup; exit state now
				}

			// *****************************
			//		Start Move to End
			// *****************************
			// Move to end has no particular movement limit, because movement is stopped by the End-of-Travel sensor
			// The only use in the finished application will be to find the ends of travel

			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE FWD TO END command, start a move
			// Forward = RIGHT/UP, positive position numbers

			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_SLEW_FWD_TO_END))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_SLEW_FWD_TO_END);			// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_SLEW_FWD_TO_END;					// move OUT, maximum possible distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				IGNORE_RETURN_VALUE SetMotionLimits(eMotor, eMoveType[eMotor]);	// calculate and set motion phase limits; no overall motion limits, so no need to check return value
				eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_FWD;	// start acceleration OUT (to right),  will end with EOT Detection (efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL)

				break;														// done with move setup; exit state now
				}


			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE REVERSE TO END command, start a move
			// REVERSE == LEFT/DOWN, Negative position numbers

			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_SLEW_REV_TO_END))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_SLEW_REV_TO_END);			// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_SLEW_REV_TO_END;					// move REVERSE, maximum possible distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				IGNORE_RETURN_VALUE SetMotionLimits(eMotor, eMoveType[eMotor]);	// calculate and set motion phase limits; no overall motion limits
				eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;	// start acceleration IN (to left), will end with EOT Detection  (efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL)
				break;														// done with move setup; exit state now
				}

			// *****************************
			//			Menu STOP
			// *****************************

			// if this is a redundant STOP command, just clear the event flag
			// this can happen during the serial menu..
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);	// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state reset
				}
	        break;


		// ==========================================================
		// END OF case ST_MTN_PHASE_FSM_STOPPED, END OF COMMAND PROCESSING
		// ==> perhaps all of the above belongs somewhere else?
		// ==========================================================


		// *************************************************
		//				ACCELERATE FORWARD
		// *************************************************
		// Forward = RIGHT/UP, positive position numbers
        case ST_MTN_PHASE_FSM_ACCELERATE_FWD:			// accelerate out (to OUT (to right))

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// if the MotionFSM has completed the required number of Motion Sensor Ticks, we are done with acceleration and MUST change states
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE))
				{
				BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE);	// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				//  lint -e788 (Info -- enum constant 'tagMoveTypes::blah' not used within defaulted switch)
				switch (eMoveType[eMotor])
					{
					case MOVE_RUN_FWD:										// RUN Forward, maximum speed at up to allowable distance
						if (pgulMSIConstantSpeedCount[eMotor] > 0)			// some constant speed phase required; pgulMSIConstantSpeedCount[] is set in Acceleration phase
							eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_FWD;	// change state to constant speed RUN phase
						else
							eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;	// no run phase, change state to start deceleration
						break;

					case MOVE_SLEW_FWD_TO_END:									// SLEW Forward to end of travel detection
							eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_FWD;	// change state to constant speed motion
						break;

					// these are all wrong, illegal, or meaningless here - but the complete list prevents a PC-Lint 788 error about not using all enums in a switch statement
					case MOVE_RUN_REV:										// RUN Reverse, maximum speed at up to maximum allowable distance
//					case MOVE_REV_STOP:										// moving IN (to left) STOP (orderly stop NOW)
					case MOVE_SLEW_REV_TO_END:								// move Reverse to end of travel detection
//					case MOVE_CENTER:										// move to previously determined center of travel
					case MOVE_NONE:											// no move, stopped
					case MOVE_STALL_RECOVERY:								// short, fast, reverse move to recover from a stall
					case MOVE_COMPLETE:
					default:
			            RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_MOVE);
						eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;	// unknown or unexpected move type; STOP
						break;
					}
				//  lint +e788

				}

			// if we get a serial menu STOP command, we are done with acceleration. Change to Deceleration for an orderly STOP
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);	// clear event flag

				// we do not really need adjust the motion profile - we just terminate the acceleration phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;	// deceleration forward, will test for forward limit (?)
				}

			break;

		// *************************************************
		//				CONSTANT SPEED FORWARD
		// *************************************************
		// Forward = RIGHT/UP, positive position numbers
       case ST_MTN_PHASE_FSM_CONSTANT_SP_FWD:				// move Forward

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// if the motion FSM has completed the Constant Speed 'run' phase, we are done with constant speed move
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
				{
				// do NOT clear the event flag here - ST_MTN_PHASE_FSM_DECELERATE_FWD needs to know how we transitioned states

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;	// deceleration forward, will test for forward limit
				break;													// no possible alternative action; exit state now
				}


			// if we get a serial menu STOP command, we are done with constant speed 'Run' move. Change to Deceleration for an orderly STOP.
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);	// clear event flag

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;	// deceleration forward, will test for forward limit
				}

			break;

		// *************************************************
		//				DECELERATE FORWARD
		// *************************************************
		// Forward = RIGHT/UP, positive position numbers
       case ST_MTN_PHASE_FSM_DECELERATE_FWD:			// decelerate Forward

			// Motion complete or any other reason to stop overrides any other state change
			// this is the PRIMARY state exit event
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;
				}

			// if we get a serial menu STOP command, there is nothing for us to do - we are already decelerating to an orderly stop
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);					// clear event flag
				}

			break;

		// end of Forward handling



		// *************************************************
		//				ACCELERATE REVERSE
		// *************************************************
		// REVERSE == LEFT/DOWN, negative position numbers
        case ST_MTN_PHASE_FSM_ACCELERATE_REV:			// accelerate REVERSE (left/up)

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// if the MotionFSM has completed the required number of Motion Sensor Ticks, we are done with acceleration
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE))
				{
				BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE);	// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				switch (eMoveType[eMotor])
					{
					case MOVE_RUN_REV:										// RUN Reverse (left/up), maximum speed at up to maximum allowable distance
						if (pgulMSIConstantSpeedCount[eMotor] > 0)			// some constant speed phase required; pgulMSIConstantSpeedCount[] is set in Acceleration phase
							eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_REV;	// constant speed motion
						else
							eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV;	// no run phase, start deceleration
						break;

					case MOVE_SLEW_REV_TO_END:								// SLEW Reverse to end of travel detection
							eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_REV;	// constant speed motion
						break;

					// these are all wrong, illegal, or meaningless here - but the complete list prevents a PC-Lint 788 error about not using all enums in a switch statement
					case MOVE_RUN_FWD:										// general move Forward
					case MOVE_SLEW_FWD_TO_END:								// move Forward to end of travel detection
					//case MOVE_CENTER:										// move to previously determined center of travel
					case MOVE_NONE:											// no move, stopped
					case MOVE_STALL_RECOVERY:								// short, fast, reverse move to recover from a stall
					case MOVE_COMPLETE:
					default:
			            RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_MOVE);
						eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;	// unknown or unexpected move type; force state change to STOP
						break;
					}

				}

			// if we get a serial menu STOP command, we are done with constant speed 'Run' move. Change to Deceleration for an orderly STOP
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);	// clear event flag

				// we do not really need adjust the motion profile - we just terminate the Acceleration phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV;	// deceleration REVERSE, will test for left limit
				}

			break;

		// *************************************************
		//				CONSTANT SPEED REVERSE
		// *************************************************
		// REVERSE == LEFT/UP, negative position numbers
       case ST_MTN_PHASE_FSM_CONSTANT_SP_REV:				// move Reverse

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// if the motion FSM has completed the Constant Speed 'run' phase, we are done with constant speed move
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
				{
				// do NOT clear the event flag here - ST_MTN_PHASE_FSM_DECELERATE_REV needs to know how we transitioned states

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV; // deceleration REVERSE, will test for reverse limit
				break;														// no possible alternative action; exit state now
				}

			// if we get a serial menu STOP command, we are done with constant speed 'Run' move. Change to Deceleration for an orderly STOP.
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);	// clear event flag

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV;// deceleration REVERSE, will test for reverse limit
				}

			break;

		// *************************************************
		//				DECELERATE REVERSE
		// *************************************************
		// REVERSE == LEFT/UP, negative position numbers
		case ST_MTN_PHASE_FSM_DECELERATE_REV:			// decelerate reverse to stop

			// Motion Complete (or any other reason to stop) overrides any other state change
			// this is the PRIMARY state exit event
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// if we get a serial menu STOP command, there is nothing for us to do - we are already decelerating to an orderly STOP
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseCommands[eMotor], EF_MTN_CMD_STOP);	// clear event flag
				}

			break;

		// end of REVERSE handling



		// *************************************************
		//	SOFT STALL (Stall Recovery)
		// *************************************************
        case ST_MTN_PHASE_FSM_SOFT_STALL:			// stalled, error recovery state

			// this is the PRIMARY state exit event
			// Stall recovery complete
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))	// EF_RESULT_STOPPED will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;		// force state change
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_STOPPED
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;				// initialize substate variables before changing states
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to HARD_STALL first
	            eMotionPhaseState[eMotor] = ST_MTN_PHASE_FSM_HARD_STALL;	// force state change
				}
	        break;

		// *************************************************
		//	HARD_STALL (NO EXIT!)
		// *************************************************
        case ST_MTN_PHASE_FSM_HARD_STALL:			// stalled, error state
			// no transition out of HARD_STALL; requires power OFF
	        break;

	    default:
            RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_STATE);
            break;

    }	// end of state transtion


	// end of state transition processing


    // ************************************************************************
    //					Process the Current or New State
    // ************************************************************************
    // State handler
    //	called one or more times for a state, depending on state implementation
    //	state transitions do NOT occur here
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate


    switch(eMotionPhaseState[eMotor])
    {

		case ST_MTN_PHASE_FSM_INIT:
	        RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_STATE);				// should never get here!
            break;

		// *************************************************
		//					STOPPED
		// *************************************************
        case ST_MTN_PHASE_FSM_STOPPED:										// all motion stopped
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// if we arrived here from a STOP command, clear the event flag
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))
						{
						BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_STOPPED);	// clear event flag
						}

					// we should only enter this substate ONCE
					if (fSubStateStatus[eMotor] IS SUBSTATE_DONE)
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
						}

					// clear any possible dangling events
					// ==> if either of these are set, they should generate a runtime error so the problem can be tracked down...
					BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE); // <sek> I think this one is problematic..
					BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE);

					#ifdef USE_DYNAMIC_LEDS
						// first entry into state, so set LED states
						SetLEDs(LEDS_STOPPED);									// turn off direction LEDs
					#endif

					// clear current move type; we are STOPPED
					eMoveType[eMotor] = MOVE_NONE;							// nominally NO move, unless we are exiting the state

					//lint -e656      // error 656: (Warning -- Arithmetic operation uses (compatible) enum's)
					// if the last move was part of the FIND_CENTER sequence, mark the eLastMove as completed
					// (NOTE: this ONLY affects the FIND_CENTER sequence, which is a sequence of states, and nothing else..)
					if (eLastMoveType[eMotor] IS_NOT MOVE_NONE)
						{
						eLastMoveType[eMotor] += MOVE_COMPLETE;
						}
					//lint +e656


					// clear flag to indicate Command has completed
					// NOTE: in order to maintain synchronization with caller (MenuFSM, MoveSequenceFSM) this must only be called ONCE
					//		that is why bMotionPhaseFSMSequenceCtr[eMotor] MUST be initialized to 0 BEFORE entry into ST_MTN_PHASE_FSM_STOPPED
					ClearCommandStarted(eMotor);

					fSubStateStatus[eMotor] = SUBSTATE_DONE;				// we are done with everything the state HAS to do
					++bMotionPhaseFSMSequenceCtr[eMotor];					// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// capture any possible multiple STOP events (this should not happen)
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOPPED))
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_STOP_CMD_OVERRUN);
						}

					// exit from state is caused by external event (efSwitchEvents, EF_xxxx_SWITCH_DOWN_EVENT), so there is nothing else to do here
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}

	        break;

		// *************************************************
		//				ACCELERATE FORWARD
		// *************************************************
        case ST_MTN_PHASE_FSM_ACCELERATE_FWD:				// accelerate Forward (to right/up)
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// first entry into state, so clear the Motion Sensor Interrupt timer (used for tracking speed)
					// timer is allowed to free-run throughout the move
					ClearMSITimer(eMotor);

					// first entry into state, so set PWM bridge direction
					PWM_SetConfig(eMotor, PWM_CONFIG_FORWARD);


					// first entry into state, so set motion command flag to start PWM
					SetMotionStarted(eMotor);								// mark motion as started, for synchronization
					BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD);

					#ifdef USE_DYNAMIC_LEDS
						// set LED display states
						SetLEDs(LEDS_FORWARD);									// set direction LEDs
					#endif

					fSubStateStatus[eMotor] = SUBSTATE_DONE;				// we are done with everything the state HAS to do
					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// check for EF_MOTION_ACCELERATE_CMD overrun 
					// MotionPhaseFSM is called every 100ms, Motion FSM is called every 25mS, so it should be processed by now..
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD))
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_ACCEL_CMD_OVERRUN);
						}

					// exit from state is caused by external event (efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE), so there is nothing else to do here
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;

		// *************************************************
		//			CONSTANT SPEED FORWARD
		// *************************************************
        case ST_MTN_PHASE_FSM_CONSTANT_SP_FWD:										// move Forward (to right/up)
			fSubStateStatus[eMotor] = SUBSTATE_DONE;								// there is actually nothing to do here; we are done with everything the state HAS to do
			break;

		// *************************************************
		//			DECELERATE FORWARD
		// *************************************************
        case ST_MTN_PHASE_FSM_DECELERATE_FWD:										// decelerate Forward (to right/up) to stop
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// if we got here because the motion FSM has completed the Constant Speed 'run' phase, 
					// there is no need to send a EF_MOTION_DECELERATE_CMD to the motion FSM
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
						{
						BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE);		// clear event flag
						}
					else
						{
						// first entry into state, so set motion command flag to force motion FSM state change
						BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD);
						}

					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// check for EF_MOTION_DECELERATE_CMD overrun 
					// User FSM is called every 100ms, Motion FSM is called every 25mS, so it should be processed by now..
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_DECEL_CMD_OVERRUN);
						}

					// exit from state is caused by external event (efMotionResultEvents[eMotor], EF_RESULT_STOPPED), so there is nothing else to do here
					// note that we do NOT set fSubStateStatus[eMotor] = SUBSTATE_DONE
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;

		// *************************************************
		//				ACCELERATE REVERSE
		// *************************************************
        case ST_MTN_PHASE_FSM_ACCELERATE_REV:										// accelerate Reverse (to left/down)
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// first entry into state, so clear the Motion Sensor Interrupt timer (used for tracking speed)
					// timer is allowed to free-run throughout the move
					ClearMSITimer(eMotor);

					// first entry into state, so set PWM bridge direction
					PWM_SetConfig(eMotor, PWM_CONFIG_REVERSE);

					// first entry into state, so set motion command flag to start PWM
					SetMotionStarted(eMotor);								// mark motion as started, for synchronization
					BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD);

					#ifdef USE_DYNAMIC_LEDS
						// set LED display states
						SetLEDs(LEDS_REVERSE);									// set direction LEDs
					#endif

					fSubStateStatus[eMotor] = SUBSTATE_DONE;				// we are done with everything the state HAS to do
					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// check for EF_MOTION_ACCELERATE_CMD overrun
					// User FSM is called every 100ms, Motion FSM is called every 25mS, so it should be processed by now..
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD))
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_ACCEL_CMD_OVERRUN);
						}

					// exit from state is caused by external event (efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE), so there is nothing else to do here
					// note that we do NOT set fSubStateStatus[eMotor] = SUBSTATE_DONE
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;

		// *************************************************
		//			CONSTANT SPEED REVERSE
		// *************************************************
        case ST_MTN_PHASE_FSM_CONSTANT_SP_REV:									// move Reverse (to left/down)
			fSubStateStatus[eMotor] = SUBSTATE_DONE;						// there is actually nothing to do here; we are done with everything the state HAS to do
			break;

		// *************************************************
		//				DECELERATE REVERSE
		// *************************************************
        case ST_MTN_PHASE_FSM_DECELERATE_REV:										// decelerate reverse to stop
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// if we got here because the motion FSM has completed the Constant Speed 'run' phase,
					// there is no need to send a EF_MOTION_DECELERATE_CMD to the motion FSM
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
						{
						BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE);		// clear event flag
						}
					else
						{
						// first entry into state, so set motion command flag to force motion FSM state change
						BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD);
						}

					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// check for EF_MOTION_DECELERATE_CMD overrun
					// MotionPhaseFSM is called every 100ms, Motion FSM is called every 25mS, so it should be processed by now..
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_DECEL_CMD_OVERRUN);
						}

					// exit from state is caused by external event (efMotionResultEvents[eMotor], EF_RESULT_STOPPED), so there is nothing else to do here
					// note that we do NOT set fSubStateStatus[eMotor] = SUBSTATE_DONE
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;


		// *************************************************
		//				SOFT STALL (recovery)
		// *************************************************
        case ST_MTN_PHASE_FSM_SOFT_STALL:			// stalled, error state
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// clear the event that sent us here..
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL))
						{
						BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL);

						// log the stall
			            RuntimeError(MTN_PHASE_FSM_ERROR_SOFT_STALL);
						}
					else
						{
						// log unexpected entry
						RuntimeError(MTN_PHASE_FSM_ERROR_UNEXPECTED_EVENT);
						}

					#ifdef USE_DYNAMIC_LEDS
						// first entry into state, so set LED states
						SetLEDs(LEDS_SOFT_STALL);								// set SOFT STALL LED
					#endif

					fSubStateStatus[eMotor] = SUBSTATE_DONE;					// we are done with everything the state HAS to do
					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;


		// *************************************************
		//				STALLED (NO EXIT!)
		// *************************************************
        case ST_MTN_PHASE_FSM_HARD_STALL:				// stalled, error state
			// no transition out of STALLED; requires power OFF
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// clear the event that sent us here..
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL))
						{
						BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL);

						// log the stall
			            RuntimeError(MTN_PHASE_FSM_ERROR_HARD_STALL);
						}
					else
						{
						// log unexpected entry
						RuntimeError(MTN_PHASE_FSM_ERROR_UNEXPECTED_EVENT);
						}

					#ifdef USE_DYNAMIC_LEDS
						// first entry into state, so set LED states
						SetLEDs(LEDS_HARD_STALL);								// set STALLED LED
					#endif

					// set flag to start Hard Stall timeout, so we recover after 
					BITSET(efTimerEvents, EF_TIMER_HARD_STALL);

					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
	        break;

	    default:
            RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_STATE);
            break;

    }	// end of state processing


	// *************************************************
	//		check for any unprocessed events
	// *************************************************
	// we will consider an event unprocessed
	// Note: this means that an asynchronous external event that was generated during THIS pass through the User FSM
	//       will NOT be consider unprocessed during THIS execution pass

	// AND User Events upon FSM entry with User Events NOW to determine what was NOT processed during this pass
	efMotionResultEventsUnprocessedThisPass = efMotionResultEventsUponEntry & efMotionResultEvents[eMotor];			// flags that were set upon entry and are STILL set now will be a 1

	if (efMotionResultEventsUnprocessedThisPass IS_NOT 0)						// any unprocessed events?
		{
		RuntimeError(MTN_PHASE_FSM_ERROR_UNPROCESSED_EVENT);							// flag runtime error
//		efUnprocessedUserEvents[eMotor] |= efMotionResultEvents[eMotor];		// keep a log of unprocessed events
		efUnprocessedUserEvents[eMotor] |= efMotionResultEventsUnprocessedThisPass;		// keep a log of unprocessed events
		}

	// some events may remain unprocessed for a few passes.. we want to track only those times when there are unprocessed events
#ifdef UNPROCESSED_EVENT_ERROR
	if (efMotionResultEvents[eMotor] IS_NOT 0)									// any unprocessed events?
		{
		RuntimeError(MTN_PHASE_FSM_ERROR_UNPROCESSED_EVENT);							// flag runtime error
		}
#endif
}

// end of MotionPhaseFSM.c

