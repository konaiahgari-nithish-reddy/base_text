// *************************************************************************************************
//										D e b o u n c e . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Switch Debounce function
//
// *************************************************************************************************
// this file is setup for 4 space indents and 8 space tabs

// -----------------------------------------------------------------------------
//								#include files
// -----------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>						// Microchip PIC32 peripheral library main header
//lint +e14

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
//#include "SmartTrak.h"			// Project wide definitions
//#include "HardwareProfile.h"

#include "EventFlags.h"			// event flags from all sources
#include "Debounce.h"			// Input Switch debounce function prototype

//#include "SerialDisplay.h"		// display functions for menus
//#include "MenuFSM.h"
//#include "StrConversions.h"		// ASCII string <==> numeric conversions

//#include "I2CBus.h"
//#include "DS3232.h"
//#include "PCA9554.h"


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------


enum tagInputSwitchErrors
{
	IS_FSM_ERROR_NONE = IS_FSM_ERROR_BASE,
	IS_FSM_UNEXPECTED_TICK,					// 1 unexpected timer tick event
	IS_FSM_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	IS_FSM_ERROR_INVALID_STATE,				// 3 not a valid state
	IS_FSM_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	IS_FSM_ERROR_SWITCH_BOUNCE				// 5 switch changed BEFORE debounce

};



// Motion Control FSM states
enum tagDebounceStates
{
    ST_IS_INIT,				// initial state, only at power up
    ST_IS_DEBOUNCE,			// debouncing input(s)
    ST_IS_STABLE			// inputs debounced, stable

};

enum tagDebounceStates eDebounceState = ST_IS_INIT;

FILE_GLOBAL	BYTE	fgcLastInputSwitches;				// most recently read state of input switches
FILE_GLOBAL	BYTE	fgcPreviousDebouncedInputSwitches;	// previous debounced state of input switches, to detect changes
FILE_GLOBAL	BYTE	fgcCurrentDebouncedInputSwitches;	// current debounced state of input switches
FILE_GLOBAL	BYTE	fgcIS_Debounce_Ctr;					// stable input state counter

#define	IS_DEBOUNCE_COUNT   5				// debounce stable input state count value; final value depends on electromechanical design of switches

//-------------------------------------------------------------------------------------------------------
// Function Bodies
//-------------------------------------------------------------------------------------------------------

// this function is called from the timer interrupt on a fixed time basis

#ifdef USE_PCA9554_IO			// enable in config.h ONLY if PCA9554 hardware is present

void Input_Debounce_FSM(BOOL fReset)
{

	BYTE cInputSwitches;
	BYTE cInputSwitchChanges;

	if (fReset IS_TRUE)				// force reset?
		{
		// initialize data
		fgcLastInputSwitches = 0;								// clear current state of input switches
		fgcPreviousDebouncedInputSwitches = 0;					// clear previous debounced switch state
		fgcCurrentDebouncedInputSwitches = 0;					// clear current debounced switch state

		eDebounceState = ST_IS_INIT;
		}

    switch(eDebounceState)
	{
		case ST_IS_INIT:			// initial state
			#ifdef USE_INPUT_SWITCH_TRIGGER
				Trigger1Level(1);									// trigger to allow viewing this event on a scope
			#endif

			fgcLastInputSwitches = GetInputSwitchState();			// read state of input switches

			#ifdef USE_INPUT_SWITCH_TRIGGER
				Trigger1Level(0);									// trigger to allow viewing this event on a scope
			#endif

			fgcIS_Debounce_Ctr = IS_DEBOUNCE_COUNT;					// initialize debounce repeat counter

			eDebounceState = ST_IS_DEBOUNCE;						// set next state
			break;

		case ST_IS_DEBOUNCE:		// debouncing input(s)
			#ifdef USE_INPUT_SWITCH_TRIGGER
				Trigger1Level(1);									// trigger to allow viewing this event on a scope
			#endif

			cInputSwitches = GetInputSwitchState();					// read input switches

			#ifdef USE_INPUT_SWITCH_TRIGGER
				Trigger1Level(0);									// trigger to allow viewing this event on a scope
			#endif

			if (cInputSwitches IS fgcLastInputSwitches)				// read current state of switches and compare to previously read value
				{
				// input switches unchanged, so bump repeat counter
				--fgcIS_Debounce_Ctr;
				if (fgcIS_Debounce_Ctr IS 0)
					{
					// switches have been stable long enough to consider debounced. update debounced switch values
					fgcPreviousDebouncedInputSwitches = fgcCurrentDebouncedInputSwitches;			// save previously debounced state
					fgcCurrentDebouncedInputSwitches = fgcLastInputSwitches;						// update current debounced state

					// ..and change to ST_IS_STABLE
					eDebounceState = ST_IS_STABLE;							// set FSM next state

					// XOR previous and current debounced states to detect bitwise differences
					// (differences will set a bit in cInputSwitchChanges)
					cInputSwitchChanges = fgcPreviousDebouncedInputSwitches ^ fgcCurrentDebouncedInputSwitches;

					// set switch event flags
   					if (cInputSwitchChanges IS_NOT ZERO)
						{

						// Move UP
						if ((cInputSwitchChanges & SWITCH1) IS_NOT ZERO)
							{
							// change on switch 1
							if ((fgcCurrentDebouncedInputSwitches & SWITCH1) IS_NOT ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_1_CLOSED_EVENT);		// switch DOWN
								}
							else
								{
								BITSET(efSwitchEvents, EF_SWITCH_1_OPEN_EVENT);		// switch UP
								}
							}

						// Move EAST
						if ((cInputSwitchChanges & SWITCH2) IS_NOT ZERO)
							{
							// change on switch 2
							if ((fgcCurrentDebouncedInputSwitches & SWITCH2) IS_NOT ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_2_CLOSED_EVENT);		// switch DOWN
								}
							else
								{
								BITSET(efSwitchEvents, EF_SWITCH_2_OPEN_EVENT);		// switch UP
								}
							}

						// move WEST
						if ((cInputSwitchChanges & SWITCH3) IS_NOT ZERO)
							{
							// change on switch 3
							if (((fgcCurrentDebouncedInputSwitches & SWITCH3) IS_NOT ZERO) AND (IS_BITCLEAR(efSwitchEvents, EF_SWITCH_3_OPEN_EVENT)))
								{
								BITSET(efSwitchEvents, EF_SWITCH_3_CLOSED_EVENT);		// switch DOWN
							//	BITCLEAR(efSwitchEvents, EF_SWITCH_3_OPEN_EVENT);		// cannot allow both DOWN and UP events simultaneously
								}
							else if ((fgcCurrentDebouncedInputSwitches & SWITCH3) IS ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_3_OPEN_EVENT);		// switch UP
								//BITCLEAR(efSwitchEvents, EF_SWITCH_3_CLOSED_EVENT);	// cannot allow both DOWN and UP events simultaneously
								}	
							}

						// Move DOWN
						if ((cInputSwitchChanges & SWITCH4) IS_NOT ZERO)
							{
							// change on switch 4
							if (((fgcCurrentDebouncedInputSwitches & SWITCH4) IS_NOT ZERO) AND (IS_BITCLEAR(efSwitchEvents, EF_SWITCH_4_OPEN_EVENT)))
								{
								BITSET(efSwitchEvents, EF_SWITCH_4_CLOSED_EVENT);		// switch DOWN
							//	BITCLEAR(efSwitchEvents, EF_SWITCH_4_OPEN_EVENT);		// cannot allow both DOWN and UP events simultaneously
								}
							else if ((fgcCurrentDebouncedInputSwitches & SWITCH4) IS ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_4_OPEN_EVENT);		// switch UP
								//BITCLEAR(efSwitchEvents, EF_SWITCH_4_CLOSED_EVENT);	// cannot allow both DOWN and UP events simultaneously
								}
							}

						if ((cInputSwitchChanges & SWITCH5) IS_NOT ZERO)
							{
							// change on switch 5
							if ((fgcCurrentDebouncedInputSwitches & SWITCH5) IS_NOT ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_5_CLOSED_EVENT);		// switch DOWN
								}
							else
								{
								BITSET(efSwitchEvents, EF_SWITCH_5_OPEN_EVENT);		// switch UP
								}
							}

						if ((cInputSwitchChanges & SWITCH6) IS_NOT ZERO)
							{
							// change on switch 6
							if ((fgcCurrentDebouncedInputSwitches & SWITCH6) IS_NOT ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_6_CLOSED_EVENT);		// switch DOWN
								}
							else
								{
								BITSET(efSwitchEvents, EF_SWITCH_6_OPEN_EVENT);		// switch UP
								}
							}

						if ((cInputSwitchChanges & SWITCH7) IS_NOT ZERO)
							{
							// change on switch 7
							if ((fgcCurrentDebouncedInputSwitches & SWITCH7) IS_NOT ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_7_CLOSED_EVENT);		// switch DOWN
								}
							else
								{
								BITSET(efSwitchEvents, EF_SWITCH_7_OPEN_EVENT);		// switch UP
								}
							}

						if ((cInputSwitchChanges & SWITCH8) IS_NOT ZERO)
							{
							// change on switch 5
							if ((fgcCurrentDebouncedInputSwitches & SWITCH8) IS_NOT ZERO)
								{
								BITSET(efSwitchEvents, EF_SWITCH_8_CLOSED_EVENT);		// switch DOWN
								}
							else
								{
								BITSET(efSwitchEvents, EF_SWITCH_8_OPEN_EVENT);		// switch UP
								}
							}

						}		// end if (cInputSwitchChanges IS_NOT ZERO)
					else
						{
						RuntimeError(IS_FSM_ERROR_UNEXPECTED_EVENT);
						}

					}		// end if (fgcIS_Debounce_Ctr IS 0)
				
				// otherwise, leave state unchanged
				}
			else
				{
				// if we were in the middle of debouncing a switch state, flag a switch bounce error
				if (fgcIS_Debounce_Ctr IS_NOT IS_DEBOUNCE_COUNT)						// has there been at least one debounce pass?
					{
					RuntimeError(IS_FSM_ERROR_SWITCH_BOUNCE);
					}

				// input switches have changed, so return to initial state
				eDebounceState = ST_IS_INIT;		// set next state
				}
			break;

		case ST_IS_STABLE:			// inputs debounced, stable
			#ifdef USE_INPUT_SWITCH_TRIGGER
				Trigger1Level(1);									// trigger to allow viewing this event on a scope
			#endif

			cInputSwitches = GetInputSwitchState();					// read input switches

			#ifdef USE_INPUT_SWITCH_TRIGGER
				Trigger1Level(0);									// trigger to allow viewing this event on a scope
			#endif

			// if the input switch state has changed return to FSM initial state
			if (cInputSwitches IS fgcCurrentDebouncedInputSwitches)
				{
				// input switches have changed, so return to initial state
				// NOTE: this is NOT cause for an event, because the switch change has NOT been debounced
				eDebounceState = ST_IS_INIT;		// set next state
				}

			// input switch value is unchanged, so there is really nothing to do here
			break;

		default:
			// we must be very lost, so just re-initialze the FSM
			eDebounceState = ST_IS_INIT;
            RuntimeError(IS_FSM_ERROR_INVALID_STATE);
	}


	// NOTE: we do not make a general check for unprocessed efSwitchEvents events here,
	// because this FSM only SETS event flags, and does not respond to them.
}

#endif		// USE_PCA9554_IO

// end of debounce.c
