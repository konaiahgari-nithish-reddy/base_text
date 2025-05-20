// *************************************************************************************************
//									P a n e l P o s i t i o n F S M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Panel Position FSM, started by motion Commands
//
//		001	16 May 13 <sek> created
//		002 20 May 13 <sek> read NV Orientation values from separate structure in RTCC RAM, display current state
//		003 21 May 13 <sek> changed order of checking for night stow, so NIGHT STOW is higher priority than HOLD
//		004 05 Jun 13 <sek> add AZ, EL offsets to soft limits
//		005 17 Aug 13 <sek> unused variable cleanup
//		006 14 Sep 13 <sek> call CurrentPosition_Set() to avoid direct access to pglCurrentPosition[eMotor]
//		007 14 Sep 13 <sek> ST_SUN_POSITION_INIT: #if defined(USE_SINGLE_POLAR_AXIS) AND defined(USE_MMA8452Q_INCLINOMETER)
//		008 19 Sep 13 <sek> use averaged inclinometer value to initialize current position
//		009 22 Sep 13 <sek> #ifdef USE_SINGLE_POLAR_AXIS ptrOrientation->fElevation = 0.0;
//		010 31 Mar 13 <sek> changed > to >= and < to <= in ST_SUN_POSITION_NIGHT_STOW, per SmartTrak
//		011 03 Apr 14 <sek> ST_SUN_POSITION_HOLD checks for elevation below Night Stow Threshold without regards for soft limits
//		012 03 Apr 14 <sek> changed to Panel Position, based on Local Coordinates
//		013 03 Apr 14 <sek> changed EF_SUN_POS_xxx to EF_PANEL_POS_xxx, changed file to PanelPositionFSM.c
//
//		AUTHOR:	    Steve Kranish	skranish@verizon.net
//					gsf Engineering	978-927-7189
//					Beverly, MA 01915
//
//		copyright (c) 2013, 2014 gsf Engineering
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

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

#include "DS3232.h"				// RTCC register level
#include "RTCC.h"				// RTCC formatting

#include "EventFlags.h"			// event flag definitions and globals

#include "SerialDisplay.h"		// display functions for menus
#include "StrConversions.h"		// ASCII string <==> numeric conversions

#include "SunPosition.h"		// Sun Position Calculations
#include "PanelPositionFSM.h"		// Sun Position FSM

//#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
//#include "MotionFSM.h"			// Motion Control function prototypes and definitions
//#include "MotionLimits.h"		// Motion limits, based on physical limitations
//#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"	// move sequence FSM

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

//#include "LEDs.h"				// LED display handler function definition, used for stall recovery states
//#include "CoordTranslate.h"		// coordinate translation functions
#include "Stubs.h"
#include "CoordTranslate.h"
#ifdef USE_MMA8452Q_INCLINOMETER
	#include "mma845x.h"              // MMA845xQ definitions
	#include "Inclinometer.h"
#endif


#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

// Everything in this file is based on LOCAL coordinates
// The only input values are:
//		ptrOrientation->fAzimuth
//		ptrOrientation->fElevation

// This should be called only AFTER:
//		SPA Calculation
//		Backtracking Adjustment
//		Convert to local coordinates

// Local coordinate system, based on facing the sun (northern hemisphere

//					  N
//				+180    -180
//     +135						-135
//
// W +90								-90 E
//  right, forward			left, reverse
//
//					  0
//					  S




//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

PRIVATE_INIT BYTE bSunPosFSMSequenceCtr = 0;				// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus = SUBSTATE_NOT_DONE;		// substate status flag

//FILE_GLOBAL SPA_MOVE_INFO fgSPAMove;

PRIVATE_INIT EVENTFLAGS efUnprocessedSunPositionEvents = 0;	// used for debugging purposes ONLY, to track unprocessed events

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

enum tagSunPositionState
{
	ST_PANEL_POS_POWER_UP,
	ST_PANEL_POS_INIT,
	ST_PANEL_POS_TRACKING,
	ST_PANEL_POS_HOLD,
	ST_PANEL_POS_NIGHT_STOW,
	ST_PANEL_POS_WIND_STOW
};

enum tagSunPositionState eSunPosState = ST_PANEL_POS_POWER_UP;			// User FSM state variable


// this is an array of pointers to state descriptor strings
// NOTE: this array must exactly track the above enum in order for it to make any sense!
FILE_GLOBAL ARRAY  char *pstrPanelPositionStateText[] =
	{
	"PowerUp",
	"Init",
	"Tracking",
	"Hold",
	"Night Stow",
	"Wind Stow",
	""
	};


 char *GetPanelPositionStateString(void)
{
	// return point to state descriptor string
	// note that we are not doing any bounds checking..
	return (pstrPanelPositionStateText[eSunPosState]);

}


// *************************************************************************************************
//							S u n P o s i t i o n F S M ( ) 
// *************************************************************************************************
// the PanelPositionFSM is called only when the MoveSequenceFSM() calls to update the SPA Calculations

enum tagPanelPositionMovement PanelPositionFSM(SmartTrakOrientation *ptrOrientation)
{

	enum tagPanelPositionMovement eRetVal = PANEL_POSITION_HOLD;// default to no move
	EVENTFLAGS  efPanelPosEventsUponEntry;				// used for debugging purposes ONLY, keeps a copy of Sun Position Event flags at entry to Motion FSM
	EVENTFLAGS  efPanelPosEventsUnprocessedThisPass;		// used for debugging purposes ONLY, AND of efPanelPosUponEntry and efPanelPos at exit from User FSM

	efPanelPosEventsUponEntry = efPanelPositionEvents;			// keep a copy of Motion Events upon entry to FSM

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

    switch(eSunPosState)
		{

		case ST_PANEL_POS_POWER_UP:			// initial state, only at power up
			eSunPosState = ST_PANEL_POS_INIT;
	        break;

		case ST_PANEL_POS_INIT:

			// check for Wind Stow
			if (IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))
				{
				// clear the calling event flag
				BITCLEAR(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW);

				eSunPosState = ST_PANEL_POS_WIND_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for elevation below Night Stow threshold
			if (ptrOrientation->fElevation <= ptrRAM_SystemParameters->fEL_NightStowThreshold)
				{
				eSunPosState = ST_PANEL_POS_NIGHT_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// clear any existing AZ and EL offset values
			// ==> <sek> why is this here? it really makes no sense; these are non-volatile system parameters
			// ptrRAM_SystemParameters->fAZ_Offset = 0.0;
			// ptrRAM_SystemParameters->fEL_Offset = 0.0;


			// check for SPA Orientation within Soft Limits
			if ((ptrOrientation->fAzimuth < (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
				OR (ptrOrientation->fAzimuth > (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset))
				#ifdef USE_ELEVATION
					OR (ptrOrientation->fElevation < (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset))
					OR (ptrOrientation->fElevation > (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset))
				#endif	// USE_ELEVATION
					)
				{
				eSunPosState = ST_PANEL_POS_HOLD;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for elevation above Night Stow threshold (sort of redundant)
			if (ptrOrientation->fElevation > ptrRAM_SystemParameters->fEL_NightStowThreshold)
				{
				eSunPosState = ST_PANEL_POS_TRACKING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// no valid transition, we are STUCK in ST_PANEL_POS_INIT
	        break;


		case ST_PANEL_POS_TRACKING:

			// check for Wind Stow
			if (IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))
				{
				// clear the calling event flag
				BITCLEAR(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW);

				eSunPosState = ST_PANEL_POS_WIND_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for elevation below Night Stow threshold
			if (ptrOrientation->fElevation <= ptrRAM_SystemParameters->fEL_NightStowThreshold)
				{
				eSunPosState = ST_PANEL_POS_NIGHT_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for SPA Orientation NOT within Soft Limits
			if ((ptrOrientation->fAzimuth <= (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
				OR (ptrOrientation->fAzimuth >= (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset))
				#ifdef USE_ELEVATION
					OR (ptrOrientation->fElevation <= (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset))
					OR (ptrOrientation->fElevation >= (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset))
				#endif	// USE_ELEVATION
					)
				{
				eSunPosState = ST_PANEL_POS_HOLD;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for elevation above Night Stow threshold
			if (ptrOrientation->fElevation > ptrRAM_SystemParameters->fEL_NightStowThreshold)
				{
				eSunPosState = ST_PANEL_POS_TRACKING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// no valid transition, stay in ST_PANEL_POS_TRACKING
	        break;


		case ST_PANEL_POS_HOLD:

			// check for Wind Stow
			if (IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))
				{
				// clear the calling event flag
				BITCLEAR(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW);

				eSunPosState = ST_PANEL_POS_WIND_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for elevation below Night Stow threshold
			if (ptrOrientation->fElevation <= ptrRAM_SystemParameters->fEL_NightStowThreshold)
				{
				eSunPosState = ST_PANEL_POS_NIGHT_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for SPA Orientation within Soft Limits AND elevation below Night Stow threshold
/*			if ( (ptrOrientation->fAzimuth > (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
				AND (ptrOrientation->fAzimuth < (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset))
				AND (ptrOrientation->fElevation > (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset))
				AND (ptrOrientation->fElevation < (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset))
				AND (ptrOrientation->fElevation <= ptrRAM_SystemParameters->fEL_NightStowThreshold) )
				{
				eSunPosState = ST_PANEL_POS_NIGHT_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}
*/
			// check for SPA Orientation within Soft Limits
			if ((ptrOrientation->fAzimuth > (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
				AND (ptrOrientation->fAzimuth < (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward  - ptrRAM_SystemParameters->fAZ_Offset))
				#ifdef USE_ELEVATION
					AND (ptrOrientation->fElevation > (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset))
					AND (ptrOrientation->fElevation < (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset))
				#endif	// USE_ELEVATION
					)
				{
				eSunPosState = ST_PANEL_POS_TRACKING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// no valid transition, stay in ST_PANEL_POS_HOLD
	        break;


		case ST_PANEL_POS_NIGHT_STOW:

			// check for Wind Stow
			if (IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))
				{
				// clear the calling event flag
				BITCLEAR(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW);

				eSunPosState = ST_PANEL_POS_WIND_STOW;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// check for elevation above Night Stow threshold
			if (ptrOrientation->fElevation > ptrRAM_SystemParameters->fEL_NightStowThreshold)
				{
				eSunPosState = ST_PANEL_POS_TRACKING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}


			// check for SPA Orientation within Soft Limits AND elevation above Night Stow threshold
			if ( (ptrOrientation->fAzimuth >= (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
				AND (ptrOrientation->fAzimuth <= (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset))
				#ifdef USE_ELEVATION
					AND (ptrOrientation->fElevation > (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset))
					AND (ptrOrientation->fElevation < (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset))
				#endif	// USE_ELEVATION
				AND (ptrOrientation->fElevation > ptrRAM_SystemParameters->fEL_NightStowThreshold) )
				{
				eSunPosState = ST_PANEL_POS_TRACKING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				break;
				}

			// any need for a transition to HOLD?

			// no valid transition, stay in ST_PANEL_POS_NIGHT_STOW
	        break;

		case ST_PANEL_POS_WIND_STOW:
			if (IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_END_WIND_STOW))
				{
				// clear the calling event flag
				BITCLEAR(efPanelPositionEvents, EF_PANEL_POS_END_WIND_STOW);

				eSunPosState = ST_PANEL_POS_TRACKING;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bSunPosFSMSequenceCtr = 0;
				}
	        break;



		}		// end    switch(eSunPosState) for state transition


    // ************************************************************************
    //					Process the Current or New State
    // ************************************************************************
    // State handler
    //	called one or more times for a state, depending on state implementation
    //	state transitions do NOT occur here
    //	no fallthrough state transitions - use fFSM_Execute flag to force immediate re-execute for eventless state changes
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate

    switch(eSunPosState)
	    {

		case ST_PANEL_POS_POWER_UP:			// initial state, only at power up
//	        RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STATE);		// should never get here!
            break;


		case ST_PANEL_POS_INIT:
			// intialize the current position (this is THE point of initialization during startup
			#if defined(USE_SINGLE_POLAR_AXIS) AND defined(USE_MMA8452Q_INCLINOMETER)
				// Called after Inclinometer averaging buffer has been initialized, so use the average value
				CurrentPosition_Set(MOTOR_AZIMUTH, ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH));
			#else
				// copy NV RAM Orientation to current Orientation counters
				// note that the MCU RAM copy of the Orientation values are copied from a separate structure in the RTCC RAM, containing ONLY the orientation,
				// rather than from the larger structure
				CurrentPosition_Set(MOTOR_AZIMUTH, ptrRTCC_RAM_MechanicalOrientation->lLastAzimuth);
				// use default return value HOLD
			#endif
			CurrentPosition_Set(MOTOR_ELEVATION, ptrRTCC_RAM_MechanicalOrientation->lLastElevation);
	        break;

		case ST_PANEL_POS_TRACKING:
			// no need to change orientation
			// ==> SmartTrak put code here to limit Orientation Azimuth to the 'Single Axis' soft limits, without considering the offset values. Seems redundant.
			eRetVal = PANEL_POSITION_MOVE;
	        break;

		case ST_PANEL_POS_HOLD:
			// change Azimuth orientation to soft limit
			if (ptrOrientation->fAzimuth <= (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
			{
				ptrOrientation->fAzimuth = (ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset);
			}
			else if (ptrOrientation->fAzimuth >= (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset))
			{
				ptrOrientation->fAzimuth = (ptrRAM_SystemParameters->fAZ_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset);
			}

			#ifdef USE_ELEVATION
				// change Elevation orientation to soft limit
				if (ptrOrientation->fElevation <= (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset))
				{
					ptrOrientation->fElevation = (ptrRAM_SystemParameters->fEL_SoftLimit_Reverse - ptrRAM_SystemParameters->fEL_Offset);
				}
				else if (ptrOrientation->fElevation >= (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset))
				{
					ptrOrientation->fElevation = (ptrRAM_SystemParameters->fEL_SoftLimit_Forward - ptrRAM_SystemParameters->fEL_Offset);
				}
			#endif	// USE_ELEVATION

			// ==> SmartTrak put code here to limit Orientation Azimuth to the 'Single Axis' soft limits, without considering the offset values. Seems redundant.

			switch(bSunPosFSMSequenceCtr)
			{
				case 0:
					// first entry into state
					eRetVal = PANEL_POSITION_MOVE;
					++bSunPosFSMSequenceCtr;
					break;

				case 1:
				default:
					// all subsequent entries into state
					eRetVal = PANEL_POSITION_HOLD;
					break;
			}
	        break;

		case ST_PANEL_POS_NIGHT_STOW:
			// change Orientation to Night Stow
			ptrOrientation->fAzimuth = ptrRAM_SystemParameters->fAZ_NightStowPosition;
			ptrOrientation->fElevation = ptrRAM_SystemParameters->fEL_NightStowPosition;
			switch(bSunPosFSMSequenceCtr)
			{
				case 0:
					// first entry into state
					eRetVal = PANEL_POSITION_MOVE;
					++bSunPosFSMSequenceCtr;
					break;

				case 1:
				default:
					// all subsequent entries into state
					eRetVal = PANEL_POSITION_HOLD;
					break;
			}
	        break;

		case ST_PANEL_POS_WIND_STOW:
			// change Orientation to Wind Stow
			ptrOrientation->fAzimuth = ptrRAM_SystemParameters->fAZ_WindStowPosition;
			ptrOrientation->fElevation = ptrRAM_SystemParameters->fEL_WindStowPosition;
			switch(bSunPosFSMSequenceCtr)
			{
				case 0:
					// first entry into state
					eRetVal = PANEL_POSITION_MOVE;
					++bSunPosFSMSequenceCtr;
					break;

				case 1:
				default:
					// all subsequent entries into state
					eRetVal = PANEL_POSITION_HOLD;
					break;
			}
	        break;

		} 	// end of state processing


	// *************************************************
	//		Handle Single Polar Axis, No Elevation
	// *************************************************

	#ifdef USE_SINGLE_POLAR_AXIS
		// single polar Axis does not handle elevation
////		ptrOrientation->fElevation = 0.0;
	#endif

	// *************************************************
	//		check for any unprocessed events
	// *************************************************
	// we will consider an event unprocessed
	// Note: this means that an asynchronous external event that was generated during THIS pass through the User FSM
	//       will NOT be consider unprocessed during THIS execution pass

	// AND Motion Events upon FSM entry with Motion Events NOW to determine what was NOT processed during this pass
	// note the we are masking out the TOO_FAST and TOO_SLOW events, which may take more than one pass to clear
	efPanelPosEventsUnprocessedThisPass = efPanelPosEventsUponEntry & efPanelPositionEvents;	// flags that were set upon entry and are STILL set now will be a 1

	if (efPanelPosEventsUnprocessedThisPass IS_NOT 0)									// any unprocessed events?
		{
//		RuntimeError(MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT);								// flag runtime error
//		efUnprocessedMoveSequenceEvents |= efMoveSequenceEvents;					// keep a log of unprocessed events
		efUnprocessedSunPositionEvents |= efPanelPosEventsUnprocessedThisPass;	// keep a log of unprocessed events
		}

#ifdef UNPROCESSED_EVENT_ERROR
	// some events may remain unprocessed for a few passes.. we want to track only those times when there are unprocessed events

	if (efMoveSequenceEvents IS_NOT 0)										// any unprocessed events?
		{
	//	RuntimeError(MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT);					// flag runtime error
		}
#endif

	AddDisplayStr("PanelPositionFSM: ");
	AddDisplayStr(GetPanelPositionStateString());
	AddDisplayNewLine();						// add line terminator
	DisplayStrWait(SERIAL_MENU_UART);				// start display (serial output) of line

	return eRetVal;
	
}


// end of PanelPositionFSM.c

