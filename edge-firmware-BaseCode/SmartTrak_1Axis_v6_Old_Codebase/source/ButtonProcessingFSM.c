// *************************************************************************************************
//							B u t t o n P r o c e s s i n g F S M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Button (Input Switch) Processing FSM, User Input
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
//#include <legacy\int_3xx_4xx_legacy.h>	// required for Input Capture interrupt handlers
//lint +e14
#include <math.h>
#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "LEDs.h"				// LED display handler function definition, used for stall recovery states
#include "SmartTrak.h"			// Project wide definitions
//#include "HardwareProfile.h"
#include "EventFlags.h"			// event flag definitions and globals

#include "MotorPWM.h"
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MoveSequenceFSM.h"
#include "MotionFSM.h"			// Motion Control function prototypes and definitions, ResetMotionFSM()
#include "SerialDisplay.h"
#include "Debounce.h"
#include "ButtonProcessingFSM.h" // Button and user input processing
#include "mma845x.h"
#include "Inclinometer.h"
#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Static Variables
//-------------------------------------------------------------------------------------------------------

PRIVATE_INIT BYTE bButtonFSMSequenceCtr = 0;				// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus = SUBSTATE_NOT_DONE;		// substate status flag

//lint -e551	error 551: (Warning -- Symbol 'efUnprocessedButtonProcessingEvents' not accessed)
PRIVATE_INIT EVENTFLAGS efUnprocessedButtonProcessingEvents = 0;	// used for debugging purposes ONLY, to track unprocessed events
//lint +e551

PRIVATE SmartTrakOrientation fgCurrentOrientation;

//-------------------------------------------------------------------------------------------------------
// Static Function Prototypes
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Function Bodies
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
//  Sun Position FSM
//
//  Input: 	    none
//  Output:	    none
//  Desc:		motion control FSM
//-------------------------------------------------------------------------------------------------------

// Command FSM states
// These states describe the Movement (or lack thereof) of the Sun Position Tracker

enum tagButtonProcessingState
{
	ST_BT_PROC_INIT,
	ST_BT_PROC_IDLE,
	ST_BT_PROC_START_MOVEMENT,
	ST_BT_PROC_START_CALIBRATE,
	ST_BT_PROC_START_SPA_TRACK,
	ST_BT_PROC_SERVICE,
	ST_BT_PROC_MOVING,
	ST_BT_PROC_STOP,
	ST_BT_PROC_RESET
};

enum tagButtonProcessingState eButtonProcessingState = ST_BT_PROC_INIT;			// FSM state variable
enum tagButtonProcessingState eServiceNextState = ST_BT_PROC_IDLE;				// flag to indicate next state after completing SERVICE



// this is an array of pointers to state descriptor strings
// NOTE: this array must exactly track the above enum in order for it to make any sense!
FILE_GLOBAL ARRAY  char *pstrButtonProcessingStateText[] =
	{
	"Init",
	"Idle",
	"Start Mvmt",
	"Start Cal",
	"Start SPA",
	"Service",
	"Moving",
	"Stop",
	"Reset",
	""
	};


 char *GetButtonProcessingStateString(void)
{
	// return point to state descriptor string
	// note that we are not doing any bounds checking..
	return (pstrButtonProcessingStateText[eButtonProcessingState]);

}


// *************************************************************************************************
//							B u t t o n P r o c e s s i n g F S M ( )
// *************************************************************************************************

void ButtonProcessingFSM()
{

	EVENTFLAGS  efButtonProcessingEventsUponEntry;				// used for debugging purposes ONLY, keeps a copy of Sun Position Event flags at entry to Motion FSM
	EVENTFLAGS  efButtonProcessingEventsUnprocessedThisPass;	// used for debugging purposes ONLY, AND of efSunPositionUponEntry and efSunPosition at exit from User FSM

	efButtonProcessingEventsUponEntry = efSwitchEvents;			// keep a copy of Motion Events upon entry to FSM
        int nStatus=0;

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

    switch(eButtonProcessingState)
		{

		case ST_BT_PROC_INIT:			// initial state, only at power up
			eButtonProcessingState = ST_BT_PROC_IDLE;
			bButtonFSMSequenceCtr = 0;
			break;

		case ST_BT_PROC_IDLE:

			// check for direction switches, only usable in MANUAL mode
			if (( (IS_BITSET(efSwitchEvents, EF_SW_UP_SWITCH_CLOSED_EVENT)) OR (IS_BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT))
					OR (IS_BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT)) OR (IS_BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT)))
                                        AND (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL))
				{
				// we do not clear the calling event flag here, because we need it in ST_BT_PROC_START_MOVEMENT to determine which direction to move
				eButtonProcessingState = ST_BT_PROC_START_MOVEMENT;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// check for Service switch (Auto/Manual)
			if (IS_BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_CLOSED_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efSwitchEvents, EF_SW_SERVICE_SWITCH_CLOSED_EVENT);
				eButtonProcessingState = ST_BT_PROC_SERVICE;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// check for Calibrate switch (Find End Points)
			if (IS_BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_CLOSED_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_CLOSED_EVENT);
				eButtonProcessingState = ST_BT_PROC_START_CALIBRATE;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// check for Virtual SPA Tracking switch
			if (IS_BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);
				eButtonProcessingState = ST_BT_PROC_START_SPA_TRACK;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// check for Reset switch
			if (IS_BITSET(efSwitchEvents, EF_SW_RESET_SWITCH_CLOSED_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efSwitchEvents, EF_SW_RESET_SWITCH_CLOSED_EVENT);
				eButtonProcessingState = ST_BT_PROC_RESET;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// no valid transition, so stay in ST_BT_PROC_IDLE
	        break;


		case ST_BT_PROC_START_MOVEMENT:
			// check for a switch OPEN event. It really does not matter which switch..
			if ((efSwitchEvents & EF_SWITCH_OPEN_EVENTS_MASK) IS_NOT 0)
			{
				efSwitchEvents = 0;										// clear calling flag(s)
				eButtonProcessingState = ST_BT_PROC_MOVING;				// switch has been released, so we can move on to MOVING
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
			}
                        // check for STOP switch (currently there is no mechanical STOP switch, this is a virtual switch from MenuFSM)
			if (IS_BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT))
				{
				efSwitchEvents = 0;										// clear calling flag(s)
				efVirtualSwitchEvents = 0;
				eButtonProcessingState = ST_BT_PROC_STOP;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}
	        break;

		case ST_BT_PROC_SERVICE:			// Auto/Manual
			// do not process state transition until state has been processed
			if (bButtonFSMSequenceCtr > 0)
			{
				// check for switch released
				if (IS_BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_OPEN_EVENT))
				{
					// clear the calling event flag
					BITCLEAR(efSwitchEvents, EF_SW_SERVICE_SWITCH_OPEN_EVENT);

					// set next state
					if (eServiceNextState IS ST_BT_PROC_IDLE)
					{
						eButtonProcessingState = ST_BT_PROC_IDLE;
					}
					else if (eServiceNextState IS ST_BT_PROC_STOP)
					{
						eButtonProcessingState = ST_BT_PROC_STOP;
					}

					fSubStateStatus = SUBSTATE_NOT_DONE;
					bButtonFSMSequenceCtr = 0;
				}
			}
	        break;

		case ST_BT_PROC_START_CALIBRATE:
			// check for switch released
			/*if (IS_BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_OPEN_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_OPEN_EVENT);
				eButtonProcessingState = ST_BT_PROC_MOVING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				//bButtonFSMSequenceCtr = 0;
				}*/
                        // check for STOP switch (currently there is no mechanical STOP switch, this is a virtual switch from MenuFSM)
			if (IS_BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT))
				{
				efSwitchEvents = 0;										// clear calling flag(s)
				efVirtualSwitchEvents = 0;
				eButtonProcessingState = ST_BT_PROC_STOP;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}
	        break;

		case ST_BT_PROC_START_SPA_TRACK:
			// check for switch released
			if (IS_BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);
				eButtonProcessingState = ST_BT_PROC_MOVING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				}
                        // check for STOP switch (currently there is no mechanical STOP switch, this is a virtual switch from MenuFSM)
			if (IS_BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT))
				{
				efSwitchEvents = 0;										// clear calling flag(s)
				efVirtualSwitchEvents = 0;
				eButtonProcessingState = ST_BT_PROC_STOP;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}
	        break;

		case ST_BT_PROC_MOVING:
			// check for Service switch (Auto/Manual)
			// this is used to exit Auto Mode while SPA tracking is running
			if (IS_BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_CLOSED_EVENT))
				{
				// clear the calling event flag
				BITCLEAR(efSwitchEvents, EF_SW_SERVICE_SWITCH_CLOSED_EVENT);
				eButtonProcessingState = ST_BT_PROC_SERVICE;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// check for STOP switch (currently there is no mechanical STOP switch, this is a virtual switch from MenuFSM)
			if (IS_BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT))
				{
				efSwitchEvents = 0;										// clear calling flag(s)
				efVirtualSwitchEvents = 0;
				eButtonProcessingState = ST_BT_PROC_STOP;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// check for any Direction keypress to STOP
			if ((efSwitchEvents & EF_MOVE_SWITCH_CLOSED_EVENTS_MASK) IS_NOT 0)
				{
				efSwitchEvents = 0;										// clear calling flag(s)
				efVirtualSwitchEvents = 0;
				eButtonProcessingState = ST_BT_PROC_STOP;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}

			// if Move Sequence is complete, we do not need to go through STOP
			if (IsMoveSequenceComplete() IS_TRUE)
				{
				eButtonProcessingState = ST_BT_PROC_IDLE;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}
	        break;

		case ST_BT_PROC_STOP:
			if (IsMoveSequenceComplete() IS_TRUE)
				{
				eButtonProcessingState = ST_BT_PROC_IDLE;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bButtonFSMSequenceCtr = 0;
				break;
				}
	        break;

		case ST_BT_PROC_RESET:
			break;

		}		// end    switch(eButtonProcessingState) for state transition


    // ************************************************************************
    //					Process the Current or New State
    // ************************************************************************
    // State handler
    //	called one or more times for a state, depending on state implementation
    //	state transitions do NOT occur here
    //	no fallthrough state transitions - use fFSM_Execute flag to force immediate re-execute for eventless state changes
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate

    switch(eButtonProcessingState)
	    {

		case ST_BT_PROC_INIT:						// initial state, only at power up
//	        RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STATE);		// should never get here!
            break;

		case ST_BT_PROC_IDLE:
			// nothing to do here; idle state
	        break;

		case ST_BT_PROC_START_MOVEMENT:

			// note that we are using the MECHANICAL orientation, which does NOT account for offsets.
			// SoftLimit does not account for offsets either, so the calculation is the same as if both accounted for offsets
			CurrentMechanicalOrientation_Read(&fgCurrentOrientation);			// get current orientation, does NOT account for offsets

			if (IS_BITSET(efSwitchEvents, EF_SW_UP_SWITCH_CLOSED_EVENT))
			{
				BITCLEAR(efSwitchEvents, EF_SW_UP_SWITCH_CLOSED_EVENT);			// clear calling flag
				SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
				// set movement distance to maximum available travel
				pgfMS_StepDistanceDegrees[MOTOR_ELEVATION] = ptrRAM_SystemParameters->fEL_SoftLimit_Forward - fgCurrentOrientation.fElevation;

				IOEXP_LED_ON(LED_UP);											// turn ON UP LED
				BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_UP);				// start move sequence
			}
			else if (IS_BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT))
			{
				BITCLEAR(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT);		// clear calling flag
				SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
				pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH] = ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse;
                                pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fgCurrentOrientation.fAzimuth - ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse;
                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                AddDisplayStr("Max East:");
                                AddDisplayStr((const char *)ftoa2(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
                                DisplayStr(SERIAL_MENU_UART);
                                #endif
				IOEXP_LED_ON(LED_EAST);											// turn ON EAST LED
				BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_RIGHT);				// start move sequence
			}
			else if (IS_BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT))
			{
				BITCLEAR(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT);		// clear calling flag
				SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
				pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH] = ptrRAM_SystemParameters->fSingle_SoftLimit_Forward;
                                pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - fgCurrentOrientation.fAzimuth;
                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                AddDisplayStr("Max West:");
                                AddDisplayStr((const char *)ftoa2(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
                                DisplayStr(SERIAL_MENU_UART);
                                #endif
                                IOEXP_LED_ON(LED_WEST);											// turn ON WEST LED
				BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_LEFT);			// start move sequence

			}
			else if (IS_BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT))
			{
				BITCLEAR(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT);			// clear calling flag
				SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
				pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH] = 0.0;
                                pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH] - fgCurrentOrientation.fAzimuth;

                                if(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] > 0)
                                    BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_LEFT);
                                else
                                    BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_RIGHT);
                                
                                pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fabs(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH]);
			}
			else
			{
				// runtime error
			}

	        break;

		/*case ST_BT_PROC_START_CALIBRATE:
			switch(bButtonFSMSequenceCtr)
			{
				case 0:
					SetMoveSequenceStarted();									// mark Move Sequence as started so we cannot misinterpret completion
					BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_FIND_END_POINTS);	// set flag to start movement sequence
					++bButtonFSMSequenceCtr;									// bump substate counter
					break;

				case 1:															// 2nd Substate to prevent duplicate start flags
				default:
					break;
			}
			break;*/

                case ST_BT_PROC_START_CALIBRATE:
                        CurrentMechanicalOrientation_Read(&fgCurrentOrientation);
			switch(bButtonFSMSequenceCtr)
			{
				case 0:
                                    if (IsMoveSequenceComplete() IS_TRUE)
                                    {
                                        MAN_EAST = 1;
                                        SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
                                        pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH] = ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse;
                                        pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fgCurrentOrientation.fAzimuth - ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse;
                                        BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_RIGHT);				// start move sequence
                                        ++bButtonFSMSequenceCtr;									// bump substate counter
                                    }
                                        break;

				case 1:															// 2nd Substate to prevent duplicate start flags
                                    if (IsMoveSequenceComplete() IS_TRUE)
                                    {
                                        MAN_WEST = 1;
                                        SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
                                        pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH] = ptrRAM_SystemParameters->fSingle_SoftLimit_Forward;
                                        pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - fgCurrentOrientation.fAzimuth;
                                        BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_LEFT);
                                        ++bButtonFSMSequenceCtr;
                                    }
                                    break;
                                case 2:															// 2nd Substate to prevent duplicate start flags
                                    if (IsMoveSequenceComplete() IS_TRUE)
                                    {
                                       bButtonFSMSequenceCtr = 0;
                                    }
                                    break;
                                default:
					break;
			}
			break;
                        
		case ST_BT_PROC_START_SPA_TRACK:
			switch(bButtonFSMSequenceCtr)
			{
				case 0:
					SetMoveSequenceStarted();								// mark Move Sequence as started so we cannot misinterpret completion
					BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_TRACK);	// set flag to start movement sequence
					++bButtonFSMSequenceCtr;								// bump substate counter
					break;

				case 1:														// 2nd Substate to prevent duplicate start flags
				default:
					break;
			}
			break;


		case ST_BT_PROC_SERVICE:
			switch(bButtonFSMSequenceCtr)
			{
				case 0:
					// toggle Tracking Mode AUTO (Tracking)/MANUAL
                    #ifdef switch_en
					if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_TRACKING)
					{
						ptrRAM_SystemParameters->ucTracking_Mode = MODE_MANUAL;

						// if we are changing from Auto (SPT Tracking) mode to Manual mode, stop all motion
						eServiceNextState = ST_BT_PROC_STOP;				// flag to indicate next state
						IOEXP_LED_OFF(LED_AUTO);							// turn OFF LED_AUTO, now in MANUAL mode
					}
					else if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
					{
						ptrRAM_SystemParameters->ucTracking_Mode = MODE_TRACKING;
						eServiceNextState = ST_BT_PROC_IDLE;				// flag to indicate next state
						IOEXP_LED_ON(LED_AUTO);								// turn ON  LED_AUTO
					}
                    #endif
					WriteFlashParameterTable();								// ==> should have a return value
					++bButtonFSMSequenceCtr;								// bump substate counter
					break;

				case 1:														// 2nd Substate to prevent duplicate actions
				default:
					break;
			}
			break;

		case ST_BT_PROC_MOVING:
			// Nothing to do here, we just stay in this state until something causes us to transition out.
	        break;

		case ST_BT_PROC_STOP:
			switch(bButtonFSMSequenceCtr)
			{
				case 0:
					// send a STOP SEQUENCE command only on the first pass through
					if (IsMoveSequenceComplete() IS_FALSE)
					{
						BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_STOP);		// bring to an orderly STOP
						ResetMotionFSM();									// prepare MotionFSM to resetart
					}

					// turn OFF all movement LEDS
					IOEXP_LED_OFF(LED_UP);									// turn OFF UP LED
					IOEXP_LED_OFF(LED_EAST);								// turn OFF EAST LED
					IOEXP_LED_OFF(LED_WEST);								// turn OFF WEST LED
					IOEXP_LED_OFF(LED_DOWN);								// turn OFF DOWN LED
					++bButtonFSMSequenceCtr;
					break;

				case 1:														// 2nd Substate to prevent duplicate actions
				default:
					break;
			}
	        break;

		case ST_BT_PROC_RESET:
			Reset();
			break;

		} 	// end of state processing


	// *************************************************
	//		check for any unprocessed events
	// *************************************************
	// we will consider an event unprocessed
	// Note: this means that an asynchronous external event that was generated during THIS pass through the User FSM
	//       will NOT be considered unprocessed during THIS execution pass

	// AND Motion Events upon FSM entry with Motion Events NOW to determine what was NOT processed during this pass
	// note the we are masking out the TOO_FAST and TOO_SLOW events, which may take more than one pass to clear
	efButtonProcessingEventsUnprocessedThisPass = efButtonProcessingEventsUponEntry & efSwitchEvents;	// flags that were set upon entry and are STILL set now will be a 1

	if (efButtonProcessingEventsUnprocessedThisPass IS_NOT 0)									// any unprocessed events?
		{
//		RuntimeError(MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT);								// flag runtime error
//		efUnprocessedMoveSequenceEvents |= efMoveSequenceEvents;					// keep a log of unprocessed events
		efUnprocessedButtonProcessingEvents |= efButtonProcessingEventsUnprocessedThisPass;	// keep a log of unprocessed events
		}

#ifdef UNPROCESSED_EVENT_ERROR
	// some events may remain unprocessed for a few passes.. we want to track only those times when there are unprocessed events

	if (efMoveSequenceEvents IS_NOT 0)										// any unprocessed events?
		{
	//	RuntimeError(MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT);					// flag runtime error
		}
#endif

}

void Switch_Init()
{
   TRISEbits.TRISE4 = 1;   LATEbits.LATE4 = 0; // Switch SW1_MODE
   TRISAbits.TRISA5 = 1;   LATAbits.LATA5 = 0;  // Switch SW2_EAST
   TRISFbits.TRISF13 = 1;    LATFbits.LATF13 = 0; // Switch SW3_WEST
}

void Switch_processing()
{
 static char  SW1_debounce_count = 0;
 static char  Sw1_Pressed = 0;
 static char  Sw1_Last = 1;

 static char  SW2_debounce_count = 0;
 static char  Sw2_Pressed = 0;
 static char  Sw2_Last = 1;

 static char  SW3_debounce_count = 0;
 static char  Sw3_Pressed = 0;
 static char  Sw3_Last = 1;

 //static BOOL MESTOP=0;
 //static BOOL MWSTOP=0;
  //DelayMs(2);

 /****************** Check SW1 *********************/
 /* debounce at least 5 iterations */
 if (SW1_MODE)  // 1 = button pressed
 {
   ++SW1_debounce_count;
 }
 else  // 0 = button released
 {
   --SW1_debounce_count;
 }

 /* see if debounce is complete */
 if (SW1_debounce_count > BOUNCE_COUNT)
 {
   Sw1_Pressed = 1;  // button pressed
   SW1_debounce_count = 0;
 }
 else if (SW1_debounce_count < -BOUNCE_COUNT)
 {
   Sw1_Pressed = 0;  // button released
   SW1_debounce_count = 0;
 }
 else
 {
   // button in transition
 }


 if (Sw1_Pressed != Sw1_Last)
 {
   Sw1_Last = Sw1_Pressed;

   /* switch changed state */
   if (Sw1_Pressed)
   {
            if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_TRACKING)
     {
         ptrRAM_SystemParameters->ucTracking_Mode = MODE_MANUAL;
         #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
            DisplayMessage(SERIAL_MENU_UART, "Manual", WAIT_FOR_DISPLAY);
         #endif
         if (IsMoveSequenceComplete() IS_FALSE)
         {
            // there is NO STOP switch at present, but with ButtonProcessingFSM() in ST_BT_PROC_MOVING,  closing ANY switch will cause a STOP
            BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
            // no need for an OPEN even, STOP clears all switch and virtual switch events
           #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                DisplayMessage(SERIAL_MENU_UART, "All Stops", WAIT_FOR_DISPLAY);
            #endif
         }
     }
     else if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
     {
         if((MAN_EAST IS_NOT 1)AND(MAN_WEST IS_NOT 1))
         {
             ptrRAM_SystemParameters->ucTracking_Mode = MODE_TRACKING;
             #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                DisplayMessage(SERIAL_MENU_UART, "Auto", WAIT_FOR_DISPLAY);
             #endif
             SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
             BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);	// start with virtual button press for ButtonProcessingFSM() handling
             BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);		// OPEN will be processed AFTER CLOSED, has the effect of push and release
         }
     }
     WriteFlashParameterTable();
   }
   else
   {
     /* do SW1 Released stuff */

   }
 }

 /****************** Check SW2 *********************/
 /* debounce at least 5 iterations */
 if (SW2_EAST)  // 1 = button pressed
 {
   ++SW2_debounce_count;
 }
 else  // 0 = button released
 {
   --SW2_debounce_count;
 }

 /* see if debounce is complete */
 if (SW2_debounce_count > BOUNCE_COUNT)
 {
   Sw2_Pressed = 1;  // button pressed
   SW2_debounce_count = 0;
 }
 else if (SW2_debounce_count < -BOUNCE_COUNT)
 {
   Sw2_Pressed = 0;  // button released
   SW2_debounce_count = 0;
 }
 else
 {
   // button in transition
 }


 if (Sw2_Pressed != Sw2_Last)
 {
   Sw2_Last = Sw2_Pressed;

   /* switch changed state */
   if (Sw2_Pressed)
   {
     /* East Switch Pressed Stuff */
     if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
       {
         if((MAN_WEST IS 0)AND (MAN_EAST IS 0))
         {
           BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
	   BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
           MAN_EAST = 1;
           #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                DisplayMessage(SERIAL_MENU_UART, "Manual East start", WAIT_FOR_DISPLAY);
           #endif
        }
        else if(MAN_EAST IS_NOT 0)
        {
            MAN_EAST = 0;
            if (IsMoveSequenceComplete() IS_FALSE)
            {
            // there is NO STOP switch at present, but with ButtonProcessingFSM() in ST_BT_PROC_MOVING,  closing ANY switch will cause a STOP
            BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
            }
            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                DisplayMessage(SERIAL_MENU_UART, "Manual East stop", WAIT_FOR_DISPLAY);
            #endif
        }
     }
   }
   else
   {
     /* do SW2 Released stuff */
   }
 }

 /****************** Check SW3 *********************/
 /* debounce at least 5 iterations */
 if (SW3_WEST)  // 1 = button pressed
 {
   ++SW3_debounce_count;
 }
 else  // 0 = button released
 {
   --SW3_debounce_count;
 }

 /* see if debounce is complete */
 if (SW3_debounce_count > BOUNCE_COUNT)
 {
   Sw3_Pressed = 1;  // button pressed
   SW3_debounce_count = 0;
 }
 else if (SW3_debounce_count < -BOUNCE_COUNT)
 {
   Sw3_Pressed = 0;  // button released
   SW3_debounce_count = 0;
 }
 else
 {
   // button in transition
 }


 if (Sw3_Pressed != Sw3_Last)
 {
   Sw3_Last = Sw3_Pressed;

   /* switch changed state */
   if (Sw3_Pressed)
   {
     /* do SW1 Pressed stuff */
       if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
       {
         if((MAN_WEST IS 0)AND (MAN_EAST IS 0))
         {
           BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
	   BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
           MAN_WEST = 1;
           #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                DisplayMessage(SERIAL_MENU_UART, "Manual West start", WAIT_FOR_DISPLAY);
            #endif
        }
        else if(MAN_WEST IS_NOT 0)
        {
            MAN_WEST = 0;
            if (IsMoveSequenceComplete() IS_FALSE)
            {
            // there is NO STOP switch at present, but with ButtonProcessingFSM() in ST_BT_PROC_MOVING,  closing ANY switch will cause a STOP
            BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
            }
            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                DisplayMessage(SERIAL_MENU_UART, "Manual West stop", WAIT_FOR_DISPLAY);
            #endif
        }
     }
   }
   else
   {
     /* do SW1 Released stuff */

   }
 }
}

// end of ButtonProcessingFSM.c


