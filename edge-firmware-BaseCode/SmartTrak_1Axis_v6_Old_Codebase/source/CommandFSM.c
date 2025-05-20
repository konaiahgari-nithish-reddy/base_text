//*************************************************************************************************
//									M o t i o n P h a s e F S M . c
//*************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Phase FSM, started by motion Commands
//
//		001	26 Mar 13 <sek> created from gsf Code Base
//		002 30 Mar 13 <sek> largely cleaned up
//		003 31 Mar 13 <sek> check for MOTOR_NONE
//		004 03 Apr 13 <sek>
//		005 04 Apr 13 <sek> changed efCommandEvents to efMotionPhaseEvents
//		006 04 Apr 13 <sek> changed from CommandFSM.c to MotionPhaseFSM.c
//
//		AUTHOR:	    Steve Kranish	skranish@verizon.net
//					gsf Engineering	978-927-7189
//					Beverly, MA 01915
//
//		copyright (c) 2013 gsf Engineering
//
//*************************************************************************************************

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

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type

#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionStats.h"		// motion statistics for reporting

#include "LEDs.h"				// LED display handler function definition, used for stall recovery states

#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// Nomenclature:

// IN == LEFT == Pushing Plunger IN
// OUT == RIGHT = Pulling Plunger OUT


enum tagCommandFSMErrors
{
	MTN_PHASE_FSM_ERROR_NONE = MTN_PHASE_FSM_ERROR_BASE,
	MTN_PHASE_FSM_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	MTN_PHASE_FSM_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	MTN_PHASE_FSM_ERROR_INVALID_STATE,			// 3 not a valid state
	MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	MTN_PHASE_FSM_ERROR_INVALID_MOVE,				// 5 not a valid move type (see enums below)
	MTN_PHASE_FSM_ERROR_SOFT_STALL,				// 6 stall recovery
	MTN_PHASE_FSM_ERROR_HARD_STALL,				// 7 motion stall error
	MTN_PHASE_FSM_ERROR_STOP_CMD_OVERRUN,			// 8 command not processed
	MTN_PHASE_FSM_ERROR_ACCEL_CMD_OVERRUN,		// 9 command not processed
	MTN_PHASE_FSM_ERROR_DECEL_CMD_OVERRUN,		// A command not processed
	MTN_PHASE_FSM_ERROR_CANNOT_FIND_CENTER,		// B incorrect location during Find Center
	MTN_PHASE_FSM_ERROR_NOT_AT_CENTER,			// C incorrect location during Find Center
											// D
											// E
	MTN_PHASE_FSM_ERROR_UNPROCESSED_EVENT = MTN_PHASE_FSM_ERROR_BASE + 0x0F
};


//-------------------------------------------------------------------------------------------------------
// Static Variables
//-------------------------------------------------------------------------------------------------------

PRIVATE_INIT BYTE bMotionPhaseFSMSequenceCtr[NUM_MOTORS] = {0, 0};								// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus[NUM_MOTORS] = {SUBSTATE_NOT_DONE, SUBSTATE_NOT_DONE};		// substate status flag
PRIVATE_INIT int fgnTotalTravel[NUM_MOTORS] = {0, 0};										// total end-to-end travel

PRIVATE_INIT EVENTFLAGS efLastMenuCommandEvent[NUM_MOTORS] = {0, 0};						// used to keep track of menu commands (specifically EF_CMD_MOVE_CENTER)

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
// These states describe the interaction between the MotionFSM and the CommandFSM
enum tagCommandStates
{
    ST_MTN_PHASE_FSM_INIT,				// initial state, only at power up

    ST_MTN_PHASE_FSM_STOPPED,				// all motion stopped

    ST_MTN_PHASE_FSM_ACCELERATE_FWD,		// accelerate forward (right, up)
    ST_MTN_PHASE_FSM_CONSTANT_SP_FWD,		// constant speed forward
	ST_MTN_PHASE_FSM_DECELERATE_FWD,		// decelerate foward to stop

    ST_MTN_PHASE_FSM_ACCELERATE_REV,		// accelerate reverse (left, down)
    ST_MTN_PHASE_FSM_CONSTANT_SP_REV,		// move to right
	ST_MTN_PHASE_FSM_DECELERATE_REV,		// decelerate left to stop

	ST_MTN_PHASE_FSM_FIND_END_FWD,		// find end of travel moving right/up
	ST_MTN_PHASE_FSM_FIND_END_REV,		// find end of travel moving left/down

//	ST_CMD_FSM_NIGHT_STOW,			// move to night stow (two axis move)
//	ST_CMD_FSM_WIND_STOW,			// move to wind stow (two axis move)

//	ST_CMD_FSM_MOVE_CENTER,			// move to center

	ST_MTN_PHASE_FSM_SOFT_STALL,			// stall recovery
	ST_MTN_PHASE_FSM_HARD_STALL			// stalled error state, no recovery possible

};

enum tagCommandStates eCommandState[NUM_MOTORS] = {ST_MTN_PHASE_FSM_INIT, ST_MTN_PHASE_FSM_INIT};			// User FSM state variable
enum tagMoveTypes eLastMoveType[NUM_MOTORS] = {MOVE_NONE, MOVE_NONE};					// used to track moves in FindEndPoints sequence


// this is an array of pointers to state descriptor strings
// NOTE: this array must exactly track the above enum in order for it to make any sense!
FILE_GLOBAL ARRAY char *pstrMotionPhaseStateText[] =
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
//	"Move Night Stow",
//	"Move Wind Stow",
//	"Move Ctr",
	"Soft Stall",
	"Hard Stall",
	""
	};


char *GetMotionPhaseStateString(enum tagMotors eMotor)
{
	// return point to state descriptor string
	// note that we are not doing any bounds checking..
	return (pstrMotionPhaseStateText[eCommandState[eMotor]]);

}



//*************************************************************************************************
//								S y n c h r o n i z a t i o n 
//*************************************************************************************************

// these functions exist to avoid the problem of synchronizing external events with move commands.
// (specifically serial Menu Commands and the Move Sequencer)
// When a process external to the CommandFSM (specifically serial Menu Commands and the Move Sequencer) starts a Command,
// it also needs to know when the Command is complete. The state of efMotionPhaseEvents is NOT an adequate indicator because
// it the initiating flag is CLEARED as soon as the command STARTS, and gives no indication of when it actually completes.
// Overall system timing (see the dispatcher in InfusionPump.c) will typically allow the external process to query 
// efMotionPhaseEvents before the Command has even started to do anyting - and interpret it as Command complete.

// so the idea here is that the external process (specifically serial Menu Commands and the Move Sequencer) calls SetCommandStarted()
// to indicate that a move is in process, setting the fgbIsMotionStarted flag to TRUE.

// When the CommandFSM enters ST_MTN_PHASE_FSM_STOPPED from some other state, the first substate of ST_MTN_PHASE_FSM_STOPPED calls ClearCommandStarted()
// to clear the fgbIsMotionStarted flag to FALSE.

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


//*************************************************************************************************
//								C o m m a n d F S M ( )
//*************************************************************************************************


/*
	FSM Events
		New Motion Command
		Overcurrent
		Overcurrent Stalled
*/

// this Finite State Machine is called repeatedly on timer ticks and input events
// execution is flow-through-and-exit; we never stay in this routine
// delays are handled by external timers and subsequent calls to this routine

void CommandFSM(enum tagMotors eMotor)
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

    switch(eCommandState[eMotor])
    {

		case ST_MTN_PHASE_FSM_INIT:					// initial state, only at power up
            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
	        break;


		//*************************************************
		//					STOPPED
		//*************************************************
        case ST_MTN_PHASE_FSM_STOPPED:				// all motion stopped

			// we are STOPPED, so we can process a new move and state change

			//*****************************
			//			STOP
			//*****************************

			// Any reason to stop overrides any other state change
			// (this may appear redundant, but has the effect of resetting the state)
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			//*****************************
			//			STALL
			//*****************************
			// STALL handling
			// NOTE: this is the only place that we can transition to SOFT_STALL or HARD_STALL
			// all other states use EF_RESULT_SOFT_STALL, EF_RESULT_HARD_STALL to transfer to here

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL))
				{
				// do not clear the event flag, it will be cleared in ST_MTN_PHASE_FSM_SOFT_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_SOFT_STALL;
				break;													// no possible alternative action; exit state now
				}

			// Stall recovery failure overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL))
				{
				// do not clear the event flag, it will be cleared in ST_MTN_PHASE_FSM_HARD_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_HARD_STALL;
				break;													// no possible alternative action; exit state now
				}


			//*****************************
			//	Check for State Complete
			//*****************************
			if (fSubStateStatus[eMotor] IS_NOT SUBSTATE_DONE)
				// state processing is not complete, so we cannot even consider a state transition
				{
				break;
				}


			// *******************************************
			//			Process Command Event Flags
			// *******************************************
			// command events may be initiated by the serial menu OR MoveSequenceFSM

			//*****************************
			//  Start Finite Distance Move
			//*****************************
			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE FWD command, start a move
			// Forward = RIGHT/UP, positive position numbers

			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_FWD))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_FWD);					// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_RUN_FWD;							// RUN Forward, maximum speed at up to maximum allowable distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				if (SetMotionLimits(eMotor, eMoveType[eMotor]) IS_TRUE)		// calculate and set motion phase limits and total motion limits
					{
					eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_FWD;		// start acceleration Forward, will test for end limit
					}
				else
					{
					eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;				// motion limits problem; cannot move (too close to end of usable travel?)
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);				// to force behavior as if TOLD to stop
					}
				break;														// done with move setup; exit state now
				}

			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE REVERSE command, start a move
			// REVERSE == LEFT/DOWN, Negative position numbers

			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_REV))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_REV);					// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_RUN_REV;							// RUN Reverse, maximum speed at up to maximum allowable distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				if (SetMotionLimits(eMotor, eMoveType[eMotor]) IS_TRUE)		// calculate and set motion phase limits and total motion limits
					{
					eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;		// start acceleration Reverse, will test for reverse limit
					}
				else
					{
					eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;					// motion limits problem; cannot move
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);		// to force behavior as if TOLD to stop
					}
				break;													// done with move setup; exit state now
				}

			//*****************************
			//		Start Move to End
			//*****************************
			// Move to end has no particular movement limit, because movement is stopped by the End-of-Travel sensor
			// The only use in the finished application will be to find the ends of travel

			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE FWD TO END command, start a move
			// Forward = RIGHT/UP, positive position numbers

			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_FWD_TO_END))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_FWD_TO_END);			// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_SLEW_FWD_TO_END;					// move OUT, maximum possible distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				IGNORE_RETURN_VALUE SetMotionLimits(eMotor, eMoveType[eMotor]);	// calculate and set motion phase limits; no overall motion limits
				eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_FWD;			// start acceleration OUT (to right),  will end with EOT Detection (efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL)

				break;														// done with move setup; exit state now
				}


			// if we have a (Serial Menu OR MoveSequenceFSM) MOVE REVERSE TO END command, start a move
			// REVERSE == LEFT/DOWN, Negative position numbers

			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_REV_TO_END))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_REV_TO_END);			// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_SLEW_REV_TO_END;					// move REVERSE, maximum possible distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				IGNORE_RETURN_VALUE SetMotionLimits(eMotor, eMoveType[eMotor]);	// calculate and set motion phase limits; no overall motion limits
				eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;			// start acceleration IN (to left), will end with EOT Detection  (efMotionEvents[eMotor], EF_MOTION_END_OF_TRAVEL)
				break;														// done with move setup; exit state now
				}

			//*****************************
			//		Menu Find Center
			//*****************************
			#ifdef NOTDEF
			// ==> not active, this really belongs in the MoveSequenceFSM

			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_FIND_END_POINTS))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_FIND_END_POINTS);			// clear event flag

				// clear ALL switch events
				efSwitchEvents = 0;

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// clear end of travel values
				pgnLeftEndOfTravel = 0;
				pgnRightEndOfTravel = 0;

				eMoveType[eMotor] = MOVE_REV_TO_END;						// move IN (to LEFT), motion stopped by End-of-Travel detection
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				eLastMoveType[eMotor] = eMoveType[eMotor];					// keep track of move type, used ONLY for Find Center sequencing

				IGNORE_RETURN_VALUE SetMotionLimits(eMotor, eMoveType[eMotor]);		// calculate and set motion phase limits; no overall motion limits
				eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;			// start acceleration IN (to left), will end with End-of-Travel detection

				break;														// done with move setup; exit state now
				}


			// ==> this really belongs in the MoveSequenceFSM

			//lint -e656      // error 656: (Warning -- Arithmetic operation uses (compatible) enum's)
			// if the previous move type was MOVE_REV_TO_END, we are in the Find Center sequence, so we now need to Slew OUT (to right)
			if (eLastMoveType[eMotor] IS (MOVE_REV_TO_END + MOVE_COMPLETE))
			//lint +e656
				{
				// clear ALL switch events; we do not need to track the EF_FIND_CENTER_SWITCH_UP_EVENT
				efSwitchEvents = 0;

				// save current position as left end of travel
				pglLeftEndOfTravel = pglCurrentPosition;

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				eMoveType[eMotor] = MOVE_FWD_TO_END;						// move to OUT (to right), maximum possible distance
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
				eLastMoveType[eMotor] = eMoveType[eMotor];					// keep track of move type, used ONLY for Find Center sequencing

				IGNORE_RETURN_VALUE SetMotionLimits(eMoveType[eMotor]);		// calculate and set motion phase limits; no overall motion limits
				eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_FWD;			// start acceleration OUT (to right), will end with End-of-Travel detection

				break;														// done with move setup; exit state now
				}

			// ==> this really belongs in the MoveSequenceFSM

			//lint -e656		error 656: (Warning -- Arithmetic operation uses (compatible) enum's)
			// if the previous move type was MOVE_SLEW_FWD, we are in the Find Center sequence, so we now need to MOVE CENTER
			if (eLastMoveType[eMotor] IS (MOVE_FWD_TO_END + MOVE_COMPLETE))
			//lint +e656
				{
				// save current position as OUT (to right) end of travel
				pglRightEndOfTravel = pglCurrentPosition;

				// calculate total travel length
				// this calculation is setup to handle all possibilities, 
				// including some which can only occur during development, with no mechanical movement restrictions.
				if ((pgnLeftEndOfTravel > 0) AND (pgnRightEndOfTravel > 0))
					{
					// both end points are positive, so we are WAY to the left..
					fgnTotalTravel[eMotor] = pgnLeftEndOfTravel - pgnRightEndOfTravel;
					}
				else if ((pgnLeftEndOfTravel < 0) AND (pgnRightEndOfTravel < 0))
					{
					// both end points are negative, so we are WAY to the right (notice that the equation is reversed!)
					fgnTotalTravel[eMotor] = ABS(pgnRightEndOfTravel) - ABS(pgnLeftEndOfTravel);
					}
				else if ((pgnLeftEndOfTravel > 0) AND (pgnRightEndOfTravel < 0))
					{
					// this is the normal case, left end positive, and right end negative
					fgnTotalTravel[eMotor] = pgnLeftEndOfTravel + ABS(pgnRightEndOfTravel);
					}
				else
					{
					// one of the values must be 0!
					fgnTotalTravel[eMotor] = ABS(pgnLeftEndOfTravel) + ABS(pgnRightEndOfTravel);
					}

				// calculate new current position
				// we are currently at the OUT (to right) end of travel
				pglCurrentPosition = -(fglTotalTravel[eMotor] / 2);

				eLastMoveType[eMotor] = MOVE_NONE;							// last part of sequence, no subsequent move

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// determine which side of (the newly set) center we are..
				if (pglCurrentPosition < 0)
					{
					// we are to the right of center, so we need to move left
					eMoveType[eMotor] = MOVE_CENTER;						// move to CENTER, going LEFT
					pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure

					if (SetMotionLimits(eMoveType[eMotor]) IS_TRUE)			// calculate and set motion phase limits and total motion limits
						{
						eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;	// start acceleration IN (to left), will test for left limit

						eMoveType[eMotor] = MOVE_RUN_REV;					// update move type, going LEFT, maximum length, stopped by EOT detection
						pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
						}
					else
						{
						eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;			// motion limits problem; cannot move
						BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);	// to force behavior as if TOLD to stop
						}
					}
				else if (pglCurrentPosition > 0)
					{
					// we are to the left of (the newly set) center, and are therefore seriously LOST
					RuntimeError(MTN_PHASE_FSM_ERROR_CANNOT_FIND_CENTER);

					eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;				// motion limits problem; cannot move
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);	// to force behavior as if TOLD to stop
					}
				else
					{
					// we are AT the center, and do not need to do anything!
					eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;				// motion limits problem; cannot move
					BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);	// to force behavior as if TOLD to stop
					}

				break;														// done with move setup; exit state now
				}
			#endif

			#ifdef NOTDEF
				//*****************************
				//			Menu CENTER
				//*****************************

				// not active
				// ==> PERHAPS this belongs here, because it is a single move

				// if we have a CENTER command, start move to center
				// this may be a Serial Menu OR MoveSequenceFSM command
				if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_CENTER))
					{
					BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_MOVE_CENTER);			// clear event flag
					BITSET(efLastMenuCommandEventl[eMotor], EF_CMD_MOVE_CENTER);		// keep track of command; cleared in ST_MTN_PHASE_FSM_STOP

					fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
					bMotionPhaseFSMSequenceCtr[eMotor] = 0;

					// determine which side of center we are..
					if (pglCurrentPosition > 0)
						{
						// we are to the left of center, so we need to move right
						// OUT == RIGHT = Pulling Plunger OUT (to right, reverse, negative position numbers) 
						eMoveType[eMotor] = MOVE_CENTER;								// move to CENTER, going OUT (to right)
						if (SetMotionLimits(eMoveType[eMotor]) IS_TRUE)					// calculate and set motion phase limits and total motion limits
							{
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_FWD;				// start acceleration OUT (to right), will test for right limit
							eMoveType[eMotor] = MOVE_FWD_TO_END;						// update move type, going OUT (RIGHT)
							pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
							}
						else
							{
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;					// motion limits problem; cannot move
							BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);		// to force behavior as if TOLD to stop
							}
						}
					else if (pglCurrentPosition < 0)
						{
						// we are to the right of center, so we need to move left
						// IN == LEFT == Pushing Plunger IN (left, forward, positive position numbers)
						eMoveType[eMotor] = MOVE_CENTER;								// move to CENTER, going IN (LEFT)
						pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure

						if (SetMotionLimits(eMoveType[eMotor]) IS_TRUE)					// calculate and set motion phase limits and total motion limits
							{
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_ACCELERATE_REV;				// start acceleration IN (to left), will test for left limit
							eMoveType[eMotor] = MOVE_REV_TO_END;							// update move type, going LEFT
							pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
							}
						else
							{
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;					// motion limits problem; cannot move
							BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);		// to force behavior as if TOLD to stop
							}
						}
					else
						{
						// we are AT the center, and do not need to do anything!
						eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;						// motion limits problem; cannot move
						BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP);			// to force behavior as if TOLD to stop
						}

					break;														// done with move setup; exit state now
					}
			#endif

			//*****************************
			//			Menu STOP
			//*****************************

			// if this is a redundant STOP command, just clear the event flag
			// this can happen during the serial menu..
			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_STOP);	// clear event flag

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				}
	        break;

		// ==========================================================
		// END OF case ST_MTN_PHASE_FSM_STOPPED, END OF COMMAND PROCESSING
		// ==> perhaps all of the above belongs somewhere else?
		// ==========================================================

		//*************************************************
		//				ACCELERATE REVERSE
		//*************************************************
		// REVERSE == LEFT/UP, negative position numbers
        case ST_MTN_PHASE_FSM_ACCELERATE_REV:			// accelerate REVERSE (left/up)

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;						// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// if the MotionFSM has completed the required number of Motion Sensor Ticks, we are done with acceleration
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE))
				{
				BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE);	// clear event flag

				switch (eMoveType[eMotor])
					{
					case MOVE_RUN_REV:										// RUN Reverse, maximum speed at up to maximum allowable distance											// move reverse (left/up)
						if (pgulMSIConstantSpeedCount[eMotor] > 0)			// some constant speed phase required; pgulMSIConstantSpeedCount[] is set in Acceleration phase
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_REV;	// constant speed motion
						else
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV;	// no run phase, start deceleration
						break;

					case MOVE_SLEW_REV_TO_END:								// SLEW Reverse to end of travel detection
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_REV;	// constant speed motion
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
						eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;			// unknown or unexpected move type; STOP
						break;
					}

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;
				}

			break;

		//*************************************************
		//					MOVE REVERSE
		//*************************************************
		// REVERSE == LEFT/UP, negative position numbers
       case ST_MTN_PHASE_FSM_CONSTANT_SP_REV:				// move Reverse

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// if the motion FSM has completed the Constant Speed 'run' phase, we are done with constant speed move
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
				{
				// do NOT clear the event flag here - ST_MTN_PHASE_FSM_DECELERATE_REV needs to know how we transitioned states

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV;					// deceleration in (to left), will test for left limit
				break;													// no possible alternative action; exit state now
				}

			// if we get a serial menu STOP command, we are done with constant speed 'Run' move
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_STOP);					// clear event flag

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_REV;					// deceleration IN (to left), will test for left limit
				}

			break;

		//*************************************************
		//				DECELERATE REVERSE
		//*************************************************
		// REVERSE == LEFT/UP, negative position numbers
		case ST_MTN_PHASE_FSM_DECELERATE_REV:			// decelerate reverse to stop

			// Motion Complete (or any other reason to stop) overrides any other state change
			// this is the PRIMARY state exit event
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				}

			break;

		// end of REVERSE handling

		//*************************************************
		//				ACCELERATE FORWARD
		//*************************************************
		// Forward = RIGHT/UP, positive position numbers
        case ST_MTN_PHASE_FSM_ACCELERATE_FWD:			// accelerate out (to OUT (to right))

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;														// no possible alternative action; exit state now
				}

			// if the MotionFSM has completed the required number of Motion Sensor Ticks, we are done with acceleration
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE))
				{
				BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE);

				//  lint -e788 (Info -- enum constant 'tagMoveTypes::blah' not used within defaulted switch)
				switch (eMoveType[eMotor])
					{
					case MOVE_RUN_FWD:											// RUN Forward, maximum speed at up to allowable distance
						if (pgulMSIConstantSpeedCount[eMotor] > 0)				// some constant speed phase required; pgulMSIConstantSpeedCount[] is set in Acceleration phase
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_FWD;	// constant speed RUN phase
						else
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;	// no run phase, start deceleration
						break;

					case MOVE_SLEW_FWD_TO_END:									// SLEW Forward to end of travel detection
							eCommandState[eMotor] = ST_MTN_PHASE_FSM_CONSTANT_SP_FWD;	// constant speed motion
						break;

					// these are all wrong, illegal, or meaningless here - but the complete list prevents a PC-Lint 788 error about not using all enums in a switch statement
					case MOVE_RUN_REV:										// RUN Reverse, maximum speed at up to maximum allowable distance
//					case MOVE_REV_STOP:										// moving IN (to left) STOP (orderly stop NOW)
					case MOVE_SLEW_REV_TO_END:									// move Reverse to end of travel detection
					//case MOVE_CENTER:										// move to previously determined center of travel
					case MOVE_NONE:											// no move, stopped
					case MOVE_STALL_RECOVERY:								// short, fast, reverse move to recover from a stall
					case MOVE_COMPLETE:
					default:
			            RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_MOVE);
						eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;					// unknown or unexpected move type; STOP
						break;
					}
				//  lint +e788

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;
				}

			break;

		//*************************************************
		//				MOVE FORWARD
		//*************************************************
		// Forward = RIGHT/UP, positive position numbers
       case ST_MTN_PHASE_FSM_CONSTANT_SP_FWD:				// move Forward

			// Any reason to stop overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// if the motion FSM has completed the Constant Speed 'run' phase, we are done with constant speed move
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE))
				{
				// do NOT clear the event flag here - ST_MTN_PHASE_FSM_DECELERATE_FWD needs to know how we transitioned states

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;					// deceleration to left, will test for left limit
				break;													// no possible alternative action; exit state now
				}


			// if we get a serial menu STOP command, we are done with constant speed 'Run' move
			// this is only used for hardware debugging and testing, to end a RUN Command
			if (IS_BITSET(efMotionPhaseEvents[eMotor], EF_CMD_STOP))
				{
				BITCLEAR(efMotionPhaseEvents[eMotor], EF_CMD_STOP);					// clear event flag

				// we do not really need adjust the motion profile - we just terminate the RUN phase, and go into the full deceleration profile..

				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_DECELERATE_FWD;					// deceleration to left, will test for left limit
				}

			break;

		//*************************************************
		//				DECELERATE FORWARD
		//*************************************************
		// Forward = RIGHT/UP, positive position numbers
       case ST_MTN_PHASE_FSM_DECELERATE_FWD:			// decelerate Forward

			// Motion complete or any other reason to stop overrides any other state change
			// this is the PRIMARY state exit event
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if ((IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_SOFT_STALL)) OR (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL)))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_XXXX_STALL
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to STOPPED first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				}

			break;

		// end of Forward handling

#ifdef NOTDEF
		//*************************************************
		//		FIND CENTER - Move IN (to left)
		//*************************************************
        case ST_MTN_PHASE_FSM_FIND_CENTER_REV:		// move to left to find center
			eMoveType[eMotor] = MOVE_REV_TO_END;		// maximum distance left to end-of-travel detection
			pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
			break;

		//*************************************************
		//		FIND CENTER - Move OUT (to right)
		//*************************************************
        case ST_MTN_PHASE_FSM_FIND_CENTER_FWD:		// move to OUT (to right) to find center
			eMoveType[eMotor] = MOVE_FWD_TO_END;	// maximum distance right to end-of-travel detection
			pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
			break;

		//*************************************************
		//				Move to CENTER
		//*************************************************
        case ST_MTN_PHASE_FSM_CONSTANT_SP_CENTER:
			eMoveType[eMotor] = MOVE_CENTER;		// move to Center
				pgMotionStats[eMotor].eMoveType = eMoveType[eMotor];		// keep a copy of move type in motion statistics structure
			break;
#endif


		//*************************************************
		//	SOFT STALL (Stall Recovery)
		//*************************************************
        case ST_MTN_PHASE_FSM_SOFT_STALL:			// stalled, error recovery state

			// this is the PRIMARY state exit event
			// Stall recovery complete
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
				{
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;					// EF_RESULT_STOP will be cleared by ST_MTN_PHASE_FSM_STOPPED state
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_STOPPED;
				break;													// no possible alternative action; exit state now
				}

			// Motion STALLED overrides any other state change
			if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_HARD_STALL))
				{
				// do not clear the event flag; it will be cleared in ST_MTN_PHASE_FSM_STOPPED
				fSubStateStatus[eMotor] = SUBSTATE_NOT_DONE;
				bMotionPhaseFSMSequenceCtr[eMotor] = 0;

				// transition to HARD_STALL first
	            eCommandState[eMotor] = ST_MTN_PHASE_FSM_HARD_STALL;
				}
	        break;

		//*************************************************
		//	HARD_STALL (NO EXIT!)
		//*************************************************
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
    //	no fallthrough state transitions - use fFSM_Execute flag to force immediate re-execute for eventless state changes
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate


    switch(eCommandState[eMotor])
    {

		case ST_MTN_PHASE_FSM_INIT:
	        RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_STATE);						// should never get here!
            break;

		//*************************************************
		//					STOPPED
		//*************************************************
        case ST_MTN_PHASE_FSM_STOPPED:											// all motion stopped
			switch(bMotionPhaseFSMSequenceCtr[eMotor])
				{
				case 0:
					// if we arrived here from a STOP command, clear the event flag
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
						{
						BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_STOP);			// clear event flag
						}

					// clear any possible dangling events
					// ==> if either of these are set, they should generate a runtime error so the problem can be tracked down...
					BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_ACCELERATION_DONE); // <sek> I think this one is problematic..
					BITCLEAR(efMotionResultEvents[eMotor], EF_RESULT_CONSTANT_SPEED_DONE);

					// first entry into state, so set LED states
					SetLEDs(LEDS_STOPPED);									// turn off direction LEDs

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

					#ifdef NOTDEF
						// if we arrived here as a result on a goto-center move, tell the goto-center FSM that we are done
						if (IS_BITSET(efLastMenuCommandEventl[eMotor], EF_CMD_MOVE_CENTER))
							{
							BITCLEAR(efLastMenuCommandEventl[eMotor], EF_CMD_MOVE_CENTER);	// clear copy of menu command

							// check for really at center
							if (ABS(pglCurrentPosition) > POSITION_CENTER_TOLERANCE)
								{
								RuntimeError(MTN_PHASE_FSM_ERROR_NOT_AT_CENTER);	// flag not really at center
								}

	/////						BITSET(efAutoCenterEvents, EF_AUTO_CENTER_AT_CENTER);		// tell AutoCenterFSM that we are done with move
							}
					#endif

					// clear flag to indicate Command has completed
					ClearCommandStarted(eMotor);

					fSubStateStatus[eMotor] = SUBSTATE_DONE;				// we are done with everything the state HAS to do
					++bMotionPhaseFSMSequenceCtr[eMotor];						// force next substate on next FSM entry
					break;

				case 1:
					// all subsequent entries into the state

					// capture any possible multiple STOP events (this should not happen)
					if (IS_BITSET(efMotionResultEvents[eMotor], EF_RESULT_STOP))
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

		//*************************************************
		//				ACCELERATE FORWARD
		//*************************************************
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
					BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD);

					// set LED display states
					SetLEDs(LEDS_FORWARD);									// set direction LEDs

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
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;

		//*************************************************
		//				MOVE FORWARD
		//*************************************************
        case ST_MTN_PHASE_FSM_CONSTANT_SP_FWD:										// move Forward (to right/up)
			fSubStateStatus[eMotor] = SUBSTATE_DONE;							// there is actually nothing to do here; we are done with everything the state HAS to do
			break;

		//*************************************************
		//			DECELERATE FORWARD
		//*************************************************
        case ST_MTN_PHASE_FSM_DECELERATE_FWD:											// decelerate Forward (to right/up) to stop
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

					// exit from state is caused by external event (efMotionResultEvents[eMotor], EF_RESULT_STOP), so there is nothing else to do here
					// note that we do NOT set fSubStateStatus[eMotor] = SUBSTATE_DONE
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;

		//*************************************************
		//				ACCELERATE REVERSE
		//*************************************************
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
					BITSET(efMotionEvents[eMotor], EF_MOTION_ACCELERATE_CMD);

					// set LED display states
					SetLEDs(LEDS_REVERSE);									// set direction LEDs

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

		//*************************************************
		//				MOVE REVERSE
		//*************************************************
        case ST_MTN_PHASE_FSM_CONSTANT_SP_REV:									// move Reverse (to left/down)
			fSubStateStatus[eMotor] = SUBSTATE_DONE;						// there is actually nothing to do here; we are done with everything the state HAS to do
			break;

		//*************************************************
		//				DECELERATE REVERSE
		//*************************************************
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
					// User FSM is called every 100ms, Motion FSM is called every 25mS, so it should be processed by now..
					if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_DECELERATE_CMD))
						{
						RuntimeError(MTN_PHASE_FSM_ERROR_DECEL_CMD_OVERRUN);
						}

					// exit from state is caused by external event (efMotionResultEvents[eMotor], EF_RESULT_STOP), so there is nothing else to do here
					// note that we do NOT set fSubStateStatus[eMotor] = SUBSTATE_DONE
					break;

				default:
					RuntimeError(MTN_PHASE_FSM_ERROR_INVALID_SUBSTATE);
					break;
				}
			break;

#ifdef NOTDEF
		//*************************************************
		//	FIND CENTER - Move Left
		//*************************************************
        case ST_MTN_PHASE_FSM_FIND_CENTER_REV:		// move to left to find center
			break;

		//*************************************************
		//	FIND CENTER - Move Right
		//*************************************************
        case ST_MTN_PHASE_FSM_FIND_CENTER_FWD:		// move to OUT (to right) to find center
			break;

		//*************************************************
		//		AUTO CENTER
		//*************************************************
        case ST_MTN_PHASE_FSM_MOVE_CENTER:			// move to center
			break;
#endif

		//*************************************************
		//				SOFT STALL (recovery)
		//*************************************************
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

					// first entry into state, so set LED states
					SetLEDs(LEDS_SOFT_STALL);								// set SOFT STALL LED

					fSubStateStatus[eMotor] = SUBSTATE_DONE;				// we are done with everything the state HAS to do
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


		//*************************************************
		//				STALLED (NO EXIT!)
		//*************************************************
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

					// first entry into state, so set LED states
					SetLEDs(LEDS_HARD_STALL);								// set STALLED LED

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


	//*************************************************
	//		check for any unprocessed events
	//*************************************************
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
#ifdef NOTDEF
	if (efMotionResultEvents[eMotor] IS_NOT 0)									// any unprocessed events?
		{
		RuntimeError(MTN_PHASE_FSM_ERROR_UNPROCESSED_EVENT);							// flag runtime error
		}
#endif
}

// end of CommandFSM.c

