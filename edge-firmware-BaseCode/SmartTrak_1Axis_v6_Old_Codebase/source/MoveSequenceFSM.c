// *************************************************************************************************
//									M o v e S e q u e n c e F S M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Phase FSM, started by motion Commands
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

#include <string.h>				// for strcpy(), strcat()
#include <math.h>				// fabs()

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

#include "DS3232.h"				// RTCC register level
#include "RTCC.h"				// RTCC formatting
#ifdef USE_MMA8452Q_INCLINOMETER
	#include "mma845x.h"         // MMA845xQ macros
	#include "Inclinometer.h"
#endif	//  USE_MMA8452Q_INCLINOMETER

#include "EventFlags.h"			// event flag definitions and globals

#include "SerialDisplay.h"		// display functions for menus
#include "StrConversions.h"		// ASCII string <==> numeric conversions
#include "ftoa.h"

#include "SunPosition.h"		// Sun Position Calculations
#include "SunPositionFSM.h"		// Sun Position FSM - operating modes
#include "PanelPositionFSM.h"
#include "MenuFSM.h"			// for orientation format functions

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"	// move sequence FSM
#include "Debounce.h"
#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

#include "LEDs.h"				// LED display handler function definition, used for stall recovery states
#include "CoordTranslate.h"		// coordinate translation functions
#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

#define	TIMER_DIVISOR			1				// timer value divisor, used for easy compile-time reduction of timer values for ease of testing and debugging

// Nomenclature:

// FORWARD == RIGHT, UP
// REVERSE == LEFT, DOWN


enum tagMoveSequenceFSMErrors
{
	MOVE_SEQ_FSM_ERROR_NONE = MOVE_SEQ_FSM_ERROR_BASE,
	MOVE_SEQ_FSM_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	MOVE_SEQ_FSM_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	MOVE_SEQ_FSM_ERROR_INVALID_STATE,			// 3 not a valid state
	MOVE_SEQ_FSM_ERROR_INVALID_SUBSTATE,		// 4 not a valid substate
	MOVE_SEQ_FSM_ERROR_INVALID_STEP,			// 5 not a valid move type (see enums below)
	MOVE_SEQ_FSM_ERROR_CANNOT_FIND_CENTER,		// 6
	MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT = MOVE_SEQ_FSM_ERROR_BASE + 0x0F		
};


// Motion Control FSM step types - these are the instructions for the Move Sequencer
enum tagMoveSequenceStepTypes
{
    MOVE_SEQ_STEP_NONE = 0,				// placekeeper
	MOVE_SEQ_STEP_START_TIMER1,			// start timer 1
	MOVE_SEQ_STEP_START_TIMER2,			// start timer 2
	MOVE_SEQ_STEP_START_TIMER3,			// start timer 3
//	MOVE_SEQ_STEP_START_COMMAND_TIMER,	// start command timer, not presently implemented
	MOVE_SEQ_STEP_WAIT_TIMER1,			// wait for timer 1 = 0
	MOVE_SEQ_STEP_WAIT_TIMER2,			// wait for timer 2 = 0
	MOVE_SEQ_STEP_WAIT_TIMER3,			// wait for timer 3 = 0
//	MOVE_SEQ_STEP_WAIT_COMMAND_TIMER,	// wait command timer, not presently implemented
	MOVE_SEQ_STEP_START_COUNTER1,		// start loop counter 1
	MOVE_SEQ_STEP_START_COUNTER2,		// start loop counter 2
	MOVE_SEQ_STEP_START_COUNTER3,		// start loop counter 3
	MOVE_SEQ_STEP_SKIP_ON_COUNTER1,		// decrement and skip next step on counter 1 = 0
	MOVE_SEQ_STEP_SKIP_ON_COUNTER2,		// decrement and skip next step on counter 2 = 0
	MOVE_SEQ_STEP_SKIP_ON_COUNTER3,		// decrement and skip next step on counter 3 = 0
	MOVE_SEQ_STEP_LABEL1,				// loop label 1
	MOVE_SEQ_STEP_LABEL2,				// loop label 2
	MOVE_SEQ_STEP_LABEL3,				// loop label 3
	MOVE_SEQ_STEP_GOTO_LABEL1,			// jump to loop label 1
	MOVE_SEQ_STEP_GOTO_LABEL2,			// jump to loop label 2
	MOVE_SEQ_STEP_GOTO_LABEL3,			// jump to loop label 3

	MOVE_SEQ_STEP_SAVE_ORIENTATION1,	// save current orientation to fgOrientation1
	MOVE_SEQ_STEP_SAVE_ORIENTATION2,	// save current orientation to fgOrientation2
	MOVE_SEQ_STEP_SAVE_ORIENTATION3,	// save current orientation to fgOrientation3
	MOVE_SEQ_STEP_RESTORE_ORIENTATION1,	// set destination to fgOrientation1
	MOVE_SEQ_STEP_RESTORE_ORIENTATION2,	// set destination to fgOrientation2
	MOVE_SEQ_STEP_RESTORE_ORIENTATION3,	// set destination to fgOrientation3

	MOVE_SEQ_STEP_EXEC_SPA_COMMAND,		// execute SPA command
	MOVE_SEQ_STEP_WAIT_SPA_COMMAND,		// wait for SPA command complete
	MOVE_SEQ_STEP_SET_SPA_MOVE_DISTANCE, // set distance for SPA move
	MOVE_SEQ_RESTORE_SUN_POSITION,		// restore stored orientation after powerup
	MOVE_SEQ_STEP_CALC_SUN_POSITION,		// calculate current Sun Orientation
//	MOVE_SEQ_STEP_SET_NIGHT_STOW_MOVE_DISTANCE, // set distance for SPA move

	MOVE_SEQ_STEP_EXEC_COMMAND,			// execute RUN or SLEW command
	MOVE_SEQ_STEP_WAIT_COMMAND,			// wait for RUN or SLEW command complete
	MOVE_SEQ_STEP_SET_MOVE_DISTANCE,	// set distance to move
	#ifdef USE_MMA8452Q_INCLINOMETER
		MOVE_SEQ_STEP_READ_INCLINOMETER,	// read inclinometer (if available)
	#endif

	MOVE_SEQ_STEP_SKIP_ON_MOVE_DONE,	// skip next step if move (angle) is complete, used for incremental moves
	MOVE_SEQ_STEP_SPA_SKIP_ON_MOVE_DONE,// adjusted for SPA sequencing

	MOVE_SEQ_STEP_CALC_CENTER,			// calculate center position

	MOVE_SEQ_SIM_INIT,					// init for SPA calculation simulation,
	MOVE_SEQ_SIM_CALC_SUN_POSITION		// calculate and display Sun Position for simulation

};



//-------------------------------------------------------------------------------------------------------
//							Move Sequence Tables
//-------------------------------------------------------------------------------------------------------

typedef struct
{
	enum	tagMoveSequenceStepTypes	eStepType;
	enum	tagAxis						eAxis;
	INT16	nVariable;									// needs to be IN32, especially for elevation linear actuator. used for move distance, command timeouts
	enum	tagMotionPhaseCommands		eCommand;
} MOVE_SEQUENCE_STEP, *PTR_MOVE_SEQUENCE_STEP;


// note use of TIMER_DIVISOR, which allows for easy compile-time shortening of all timers for testing

// Stopped sequence, termination only
FILE_GLOBAL_INIT ARRAY  MOVE_SEQUENCE_STEP fg_Idle_Sequence[] =
{
	{ MOVE_SEQ_STEP_NONE,			AXIS_NONE, 0,		EF_MTN_CMD_NONE}			// terminator, was EF_MTN_CMD_STOP
};

FILE_GLOBAL_INIT ARRAY  MOVE_SEQUENCE_STEP fg_Stopped_Sequence[] =
{

#ifdef USE_AZIMUTH
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_STOP},			// stop Azimuth
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete
#endif	// USE_AZIMUTH

#ifdef USE_ELEVATION
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION, 0,	EF_MTN_CMD_STOP},			// stop Elevation
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// wait for move to complete
#endif	// USE_ELEVATION
	{ MOVE_SEQ_STEP_NONE,			AXIS_NONE, 0,		EF_MTN_CMD_NONE}			// terminator, was EF_MTN_CMD_STOP
};


// *************************************
//			Move Command Sequences
// *************************************
#ifdef USE_AZIMUTH
	// Run Right (FORWARD)
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_RunRight_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_FWD},		// move FORWARD (Right)
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};

	// Run Left (REVERSE)
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_RunLeft_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_REV},		// move REVERSE (left)
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};
#endif	//  USE_AZIMUTH


#ifdef USE_ELEVATION
	// Run Up (FORWARD)
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_RunUp_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_RUN_FWD},		// move FORWARD (Up)
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};

	// Run Down (REVERSE)
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_RunDown_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_RUN_REV},		// move REVERSE (Down)
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};
#endif	// USE_ELEVATION


// *************************************
//	Incremental Move Command Sequences
// *************************************
#ifdef USE_AZIMUTH
	// Incremental Run Right (FORWARD)
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_IncrementalRunRight_Sequence[] =
	{
		{MOVE_SEQ_STEP_SAVE_ORIENTATION1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},	// save current orientation to fgOrientation1

		{ MOVE_SEQ_STEP_READ_INCLINOMETER,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer (if available)

		{ MOVE_SEQ_STEP_LABEL1,				AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// loop label

			{ MOVE_SEQ_STEP_EXEC_COMMAND,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_FWD},		// move FORWARD (Right)
			{ MOVE_SEQ_STEP_WAIT_COMMAND,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

			{ MOVE_SEQ_STEP_READ_INCLINOMETER,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer (if available)

			{MOVE_SEQ_STEP_SKIP_ON_MOVE_DONE,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_FWD},		// decrement and skip next step if move complete (EF_MTN_CMD_RUN_FWD flag to indication direction ONLY)

		// bottom of outer loop
		{ MOVE_SEQ_STEP_GOTO_LABEL1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop to label1

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};

	// Run Left (REVERSE)
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_IncrementalRunLeft_Sequence[] =
	{
		{MOVE_SEQ_STEP_SAVE_ORIENTATION1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},	// save current orientation to fgOrientation1

		{ MOVE_SEQ_STEP_READ_INCLINOMETER,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer (if available)

		{ MOVE_SEQ_STEP_LABEL1,				AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// loop label

			{ MOVE_SEQ_STEP_EXEC_COMMAND,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_REV},		// move FORWARD (Right)
			{ MOVE_SEQ_STEP_WAIT_COMMAND,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

			{ MOVE_SEQ_STEP_READ_INCLINOMETER,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer (if available)

			{MOVE_SEQ_STEP_SKIP_ON_MOVE_DONE,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_REV},		// decrement and skip next step if move complete (EF_MTN_CMD_RUN_REV flag to indication direction ONLY)

		// bottom of outer loop
		{ MOVE_SEQ_STEP_GOTO_LABEL1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop to label1

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};
#endif	//  USE_AZIMUTH


// *************************************
//	Slew-to-End Command Sequences
// *************************************
#ifdef USE_AZIMUTH
	// Slew Right (FORWARD) to end
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SlewRightToEnd_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_SLEW_FWD_TO_END},	// run FORWARD (Right) to end, half speed
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};

	// Slew Left (REVERSE) to end
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SlewLeftToEnd_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_SLEW_REV_TO_END},	// run REVERSE (left) to end, half speed
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};
#endif	// USE_AZIMUTH


#ifdef USE_ELEVATION
	// Slew Up (FORWARD) to end
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SlewUpToEnd_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_SLEW_FWD_TO_END},	// run FORWARD (Up) to end, half speed
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};

	// Slew Down (REVERSE) to end
	FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SlewDownToEnd_Sequence[] =
	{
		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_SLEW_REV_TO_END},	// run REVERSE (down) to end, half speed
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		20,	EF_MTN_CMD_NONE},			// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,		0,	EF_MTN_CMD_NONE}			// terminator
	};
#endif	// USE_ELEVATION
// *************************************
//	Find End Points Command Sequence
// *************************************
FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_FindEndPoints_Sequence[] =
{

#ifdef USE_AZIMUTH
	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_AZIMUTH,	AZ_MSI_TICKS_PER_SLEW, EF_MTN_CMD_NONE},	// set maximum distance for slew
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_SLEW_FWD_TO_END},	// Slew FORWARD (Right) to Limit Switch, half speed. Can potentially slew 270 degrees
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_NONE},				// wait for move to complete

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			10,			EF_MTN_CMD_NONE},				// start 1 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 1 second timer to complete

	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_AZIMUTH,	(AZ_TICKS_EOT_MARGIN/2), EF_MTN_CMD_NONE},	// set distance to move away from EOT detection
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_RUN_REV},			// move REVERSE (Left) for 2.5 degrees
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_NONE},				// wait for move to complete
	{ MOVE_SEQ_STEP_SAVE_ORIENTATION1,	AXIS_NONE,		0,			EF_MTN_CMD_NONE},				// save Forward End of Travel orientation

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			20,			EF_MTN_CMD_NONE},				// start 2 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 2 second timer to complete

	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_AZIMUTH,	-AZ_MSI_TICKS_PER_SLEW, EF_MTN_CMD_NONE},	// set maximum distance for slew
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_SLEW_REV_TO_END},	// run REVERSE (left) to Limit Switch, half speed.  Can potentially slew 265 degrees.
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_NONE},				// wait for move to complete

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			10,			EF_MTN_CMD_NONE},				// start 1 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 1 second timer to complete

	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_AZIMUTH,	(AZ_TICKS_EOT_MARGIN/2), EF_MTN_CMD_NONE},	// set distance to move away from EOT detection
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_RUN_FWD},			// move FORWARD (right) for 2.5 degrees
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_NONE},				// wait for move to complete
	{ MOVE_SEQ_STEP_SAVE_ORIENTATION2,	AXIS_NONE,		0,			EF_MTN_CMD_NONE},				// save Reverse End of Travel orientation

	// calculate center of travel, Orientation1 to Orientation2
	{ MOVE_SEQ_STEP_CALC_CENTER,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_NONE},				// calculate center point
#endif	// USE_AZIMUTH

#if defined(USE_AZIMUTH) AND defined (USE_ELEVATION)
	#ifdef	RETURN_TO_CENTER
		// return to original location
		{ MOVE_SEQ_STEP_RESTORE_ORIENTATION1,	AXIS_NONE,	0,			EF_MTN_CMD_NONE},				// set newly calculated center point as destination

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			20,			EF_MTN_CMD_NONE},				// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_RUN_FWD},			// move FORWARD (Right) to AZIMUTH CENTER
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,		0,			EF_MTN_CMD_NONE},				// wait for move to complete
	#endif	//	RETURN_TO_CENTER

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			20,			EF_MTN_CMD_NONE},				// start 2 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 2 second timer to complete
#endif	// defined(USE_AZIMUTH) AND defined (USE_ELEVATION)


#ifdef USE_ELEVATION
	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_ELEVATION,	EL_MSI_TICKS_PER_SLEW, EF_MTN_CMD_NONE},	// set maximum distance for slew
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_SLEW_FWD_TO_END},	// run FORWARD (Up) to end, half speed
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_NONE},				// wait for move to complete

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			10,			EF_MTN_CMD_NONE},				// start 1 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 1 second timer to complete

	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_ELEVATION,	(EL_TICKS_EOT_MARGIN/2), EF_MTN_CMD_NONE},	// set distance to move away from EOT detection
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_RUN_REV},			// move REVERSE (down) for 2.5 degrees
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_NONE},				// wait for move to complete
	{ MOVE_SEQ_STEP_SAVE_ORIENTATION1,	AXIS_NONE,		0,			EF_MTN_CMD_NONE},				// save Forward End of Travel orientation

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			20,			EF_MTN_CMD_NONE},				// start 2 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 2 second timer to complete

	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_ELEVATION,	-EL_MSI_TICKS_PER_SLEW, EF_MTN_CMD_NONE},	// set maximum distance for slew
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_SLEW_REV_TO_END},	// run REVERSE (down) to Limit Switch, half speed.
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_NONE},				// wait for move to complete

	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			10,			EF_MTN_CMD_NONE},				// start 1 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 1 second timer to complete

	{ MOVE_SEQ_STEP_SET_MOVE_DISTANCE, AXIS_ELEVATION,	(EL_TICKS_EOT_MARGIN/2), EF_MTN_CMD_NONE},	// set distance to move away from EOT detection
	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_RUN_FWD},			// move FORWARD (up) for 2.5 degrees
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_NONE},				// wait for move to complete
	{ MOVE_SEQ_STEP_SAVE_ORIENTATION2,	AXIS_NONE,		0,			EF_MTN_CMD_NONE},				// save Reverse End of Travel orientation

	// calculate center of travel, Orientation1 to Orientation2
	{ MOVE_SEQ_STEP_CALC_CENTER,	AXIS_ELEVATION,		0,			EF_MTN_CMD_NONE},				// calculate center point

	#ifdef	RETURN_TO_CENTER
		// return to original location
		{ MOVE_SEQ_STEP_RESTORE_ORIENTATION1,	AXIS_NONE,	0,			EF_MTN_CMD_NONE},				// set newly calculated center point as destination

		{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			20,			EF_MTN_CMD_NONE},				// start 2 second timer
		{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 2 second timer to complete

		{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_RUN_FWD},			// move FORWARD (UP) to ELEVATION CENTER
		{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_ELEVATION,		0,			EF_MTN_CMD_NONE},				// wait for move to complete
	#endif	//	RETURN_TO_CENTER


	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,			20,			EF_MTN_CMD_NONE},				// start 2 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,			0,			EF_MTN_CMD_NONE},				// wait for 2 second timer to complete
#endif	// USE_ELEVATION

	{ MOVE_SEQ_STEP_NONE,			AXIS_NONE,			0,			EF_MTN_CMD_NONE}			// terminator
};


// this is a simplistic simulation of SPA movement. It does NOT make use of SPA calculations.
// AZIMUTH Axis ONLY
// Sequence:
//		save initial orientation
//			move 1 step forward
//			delay
//			repeat 25 times
//		return to initial orientation
FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SPASimulation_Sequence[] =
{
	{ MOVE_SEQ_STEP_SAVE_ORIENTATION1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// save initial orientation
	// top of loop for accelerated SPA Track Simulation
	{ MOVE_SEQ_STEP_START_COUNTER1,		AXIS_NONE,		25,	EF_MTN_CMD_NONE},		// start (outer) loop counter 1, 25 cycles total
		{ MOVE_SEQ_STEP_LABEL1,			AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop label

			// move one step
			{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_FWD},		// move FORWARD (Right)
			{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete
			{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		100, EF_MTN_CMD_NONE},			// start 10 second timer
			{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 10 second timer to complete

		// bottom of outer loop
		{ MOVE_SEQ_STEP_SKIP_ON_COUNTER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// decrement Counter1, loop exit on Counter1 == 0
		{ MOVE_SEQ_STEP_GOTO_LABEL1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop to label1

	// return to original location
	{ MOVE_SEQ_STEP_RESTORE_ORIENTATION1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},	// restore initial orientation

	{ MOVE_SEQ_STEP_EXEC_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_RUN_REV},		// move REVERSE (Left) to return to starting point
	{ MOVE_SEQ_STEP_WAIT_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete
	{ MOVE_SEQ_STEP_START_TIMER1,	AXIS_NONE,		10, EF_MTN_CMD_NONE},			// start 1 second timer
	{ MOVE_SEQ_STEP_WAIT_TIMER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 1 second timer to complete

	{ MOVE_SEQ_STEP_NONE,		AXIS_NONE,		0,		EF_MTN_CMD_NONE}			// terminator
};

// test for SPA calculations, without any movement
// calculate Sun Position every 2 minutes for 18 hours
FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SPACalc_Sequence[] =
{
	{ MOVE_SEQ_STEP_SAVE_ORIENTATION1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// save initial orientation
	{ MOVE_SEQ_SIM_INIT,				AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// initialize Simulation DateTime
	// top of loop for SPA Calculations
	{ MOVE_SEQ_STEP_START_COUNTER1,			AXIS_NONE,		1440, EF_MTN_CMD_NONE},	// start (outer) loop counter 1, 1440 cycles total, simulates 24 hours at 1 minute intervals
		{ MOVE_SEQ_STEP_LABEL1,				AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop label

			{ MOVE_SEQ_SIM_CALC_SUN_POSITION,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// calculate current Sun Position
//			{ MOVE_SEQ_STEP_START_TIMER1,		AXIS_NONE,		10, EF_MTN_CMD_NONE},		// start 1 second timer
//			{ MOVE_SEQ_STEP_WAIT_TIMER1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// wait for 1 second timer to complete

		// bottom of outer loop
		{ MOVE_SEQ_STEP_SKIP_ON_COUNTER1,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// decrement Counter1, loop exit on Counter1 == 0
		{ MOVE_SEQ_STEP_GOTO_LABEL1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop to label1

	{ MOVE_SEQ_STEP_NONE,				AXIS_NONE,		0,	EF_MTN_CMD_NONE}		// terminator
};


// Single Axis SPA Tracking Sequence
FILE_GLOBAL_INIT ARRAY  const MOVE_SEQUENCE_STEP fg_SPATrack_Sequence[] =
{
	{ MOVE_SEQ_RESTORE_SUN_POSITION,	AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// recover saved orientation from NV RTCC RAM

	// top of loop for  SPA Tracking
	{ MOVE_SEQ_STEP_LABEL1,				AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// loop label

		{ MOVE_SEQ_STEP_START_TIMER1,			AXIS_NONE,		600, EF_MTN_CMD_NONE},			// start 120 second timer

		// move calculated distance of one step
		{ MOVE_SEQ_STEP_CALC_SUN_POSITION,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// calculate current Sun Position for both axes
		#ifdef USE_AZIMUTH
			#ifdef USE_COMPLETE_MOVES
				{ MOVE_SEQ_STEP_SET_SPA_MOVE_DISTANCE,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},		// set distance for SPA move
				{ MOVE_SEQ_STEP_EXEC_SPA_COMMAND,		AXIS_AZIMUTH,	2400,	EF_MTN_CMD_RUN},	// move, direction determined at run time, timeout 360 seconds
				{ MOVE_SEQ_STEP_WAIT_SPA_COMMAND,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},		// wait for move to complete
				#ifdef USE_MMA8452Q_INCLINOMETER
					{ MOVE_SEQ_STEP_READ_INCLINOMETER,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},		// read inclinometer (if available)
				#endif
			#endif	// USE_COMPLETE_MOVES

			#ifdef USE_INCREMENTAL_MOVES
				{ MOVE_SEQ_STEP_SET_SPA_MOVE_DISTANCE,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},		// set distance for SPA move
				{ MOVE_SEQ_STEP_SAVE_ORIENTATION1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// save current orientation to fgOrientation1

				{ MOVE_SEQ_STEP_READ_INCLINOMETER,		AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer (if available)

				{ MOVE_SEQ_STEP_LABEL2,					AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// loop label

					{ MOVE_SEQ_STEP_EXEC_SPA_COMMAND,	AXIS_AZIMUTH,	3600,	EF_MTN_CMD_RUN},		// move FORWARD (Right)
					{ MOVE_SEQ_STEP_WAIT_COMMAND,		AXIS_AZIMUTH,		0,	EF_MTN_CMD_NONE},		// wait for increment to complete (NOT end of entire move)
					{ MOVE_SEQ_STEP_START_TIMER2,		AXIS_NONE,		20, EF_MTN_CMD_NONE},			// start 2 second timer, to make sure motion has stopped
					{ MOVE_SEQ_STEP_WAIT_TIMER2,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 2 second timer to complete, to make sure motion has stopped

					//{ MOVE_SEQ_STEP_READ_INCLINOMETER,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer (if available)

					{MOVE_SEQ_STEP_SPA_SKIP_ON_MOVE_DONE, AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// read inclinometer, decrement and skip next step if move complete

				// bottom of outer loop
				{ MOVE_SEQ_STEP_GOTO_LABEL2,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// loop to label1

				{ MOVE_SEQ_STEP_WAIT_SPA_COMMAND,	AXIS_AZIMUTH,	0,	EF_MTN_CMD_NONE},			// wait for move to complete

			#endif	// USE_INCREMENTAL_MOVES

		#endif	// USE_AZIMUTH

		#if defined(USE_AZIMUTH) AND defined (USE_ELEVATION)
			{ MOVE_SEQ_STEP_START_TIMER2,			AXIS_NONE,		10,	EF_MTN_CMD_NONE},			// start 1 second timer (delay between two axis moves)
			{ MOVE_SEQ_STEP_WAIT_TIMER2,			AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 1 second timer to complete
		#endif		// defined(USE_AZIMUTH) AND defined (USE_ELEVATION)

		#ifdef USE_ELEVATION
			{ MOVE_SEQ_STEP_SET_SPA_MOVE_DISTANCE,	AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// set distance for SPA move
			{ MOVE_SEQ_STEP_EXEC_SPA_COMMAND,		AXIS_ELEVATION,	3600,	EF_MTN_CMD_RUN},		// move, direction determined at run time, timeout 360 seconds
			{ MOVE_SEQ_STEP_WAIT_SPA_COMMAND,		AXIS_ELEVATION,	0,	EF_MTN_CMD_NONE},			// wait for move to complete
		#endif		// USE_ELEVATION
		{ MOVE_SEQ_STEP_WAIT_TIMER1,			AXIS_NONE,		0,	EF_MTN_CMD_NONE},			// wait for 120 second timer to complete

	// bottom of outer loop
	{ MOVE_SEQ_STEP_GOTO_LABEL1,		AXIS_NONE,		0,	EF_MTN_CMD_NONE},		// loop to label1

	{ MOVE_SEQ_STEP_NONE,				AXIS_NONE,		0,	EF_MTN_CMD_NONE}		// terminator
};


// we need to keep track of TWO pointers into sequence tables.

//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

PRIVATE_INIT BYTE bMoveSeqFSMSequenceCtr = 0;				// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus = SUBSTATE_NOT_DONE;		// substate status flag
PRIVATE_INIT UINT32 fgulTotalTravel[NUM_MOTORS] = {0, 0};	// total end-to-end travel

// PRIVATE_INIT float fgfMoveIncrementDegrees[NUM_MOTORS] = {0.0, 0.0};
#define	MOVE_INCREMENT_COUNT		2
#define	MOVE_INCREMENT_COUNT_FLOAT	2.0

//PRIVATE_INIT EVENTFLAGS fgefMoveSequenceEventsCopy = 0;		// copy of event flags, for debugging, never accessed by software

//lint -esym(552,efUnprocessedMoveSequenceEvents)		error 552: (Warning -- Symbol 'efUnprocessedMoveSequenceEvents' not accessed)
PRIVATE_INIT EVENTFLAGS efUnprocessedMoveSequenceEvents = 0;	// used for debugging purposes ONLY, to track unprocessed events

// pointer to current MoveSequenceStep array
FILE_GLOBAL MOVE_SEQUENCE_STEP * fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_Idle_Sequence[0];

PRIVATE_INIT WORD fgwTimer1 = 0;
PRIVATE_INIT WORD fgwTimer2 = 0;
PRIVATE_INIT WORD fgwTimer3 = 0;
PRIVATE_INIT WORD fgwCmdTimer = 0;

PRIVATE_INIT WORD fgwCounter1 = 0;
PRIVATE_INIT WORD fgwCounter2 = 0;
PRIVATE_INIT WORD fgwCounter3 = 0;

FILE_GLOBAL MOVE_SEQUENCE_STEP * fgptrLabel1 = (PTR_MOVE_SEQUENCE_STEP)&fg_Idle_Sequence[0];
FILE_GLOBAL MOVE_SEQUENCE_STEP * fgptrLabel2 = (PTR_MOVE_SEQUENCE_STEP)&fg_Idle_Sequence[0];
FILE_GLOBAL PTR_MOVE_SEQUENCE_STEP fgptrLabel3 = (PTR_MOVE_SEQUENCE_STEP)&fg_Idle_Sequence[0];

PRIVATE_INIT WORD fgwCmdExecCounter = 0;

PRIVATE SmartTrakOrientation fgOrientation1;						// saved orientations
PRIVATE SmartTrakOrientation fgOrientation2;
PRIVATE SmartTrakOrientation fgOrientation3;
PRIVATE SmartTrakOrientation fgCurrentLocalOrientation;				// local orientation accounts for offsets
PRIVATE SmartTrakOrientation fgSPAOrientation;						// output of SPA Calculations

FILE_GLOBAL  RTCC_DATE_TIME SimDateTime;

FILE_GLOBAL SPA_MOVE_INFO fgSPAMove;

FILE_GLOBAL INT32	fglRightEndOfTravel = 0;
FILE_GLOBAL INT32	fglLeftEndOfTravel = 0;
FILE_GLOBAL INT32	fglUpEndOfTravel = 0;
FILE_GLOBAL INT32	fglDownEndOfTravel = 0;

//PRIVATE_INIT float  fgdefferenceAngle = 0.0;
int nStatus = 0;
FILE_GLOBAL  int Buid_Diff_Date;
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
enum tagMoveSeqStates
{
    ST_MOVE_SEQ_INIT,			// initial state, only at power up

    ST_MOVE_SEQ_STOPPED,		// all motion stopped

    ST_MOVE_SEQ_ACTIVE			// move sequence underway

};

enum tagMoveSeqStates eMoveSeqState = ST_MOVE_SEQ_INIT;			// User FSM state variable
//enum tagMoveTypes eLastMoveType = MOVE_NONE;			// used to track moves in FindCenter sequence

enum tagPanelPositionMovement eMoveState = PANEL_POSITION_HOLD;
enum tagPanelPositionMovement eTrackState = PANEL_POSITION_HOLD;

// this is an array of pointers to state descriptor strings
// NOTE: this array must exactly track the above enum in order for it to make any sense!
FILE_GLOBAL ARRAY  char *pstrMoveSeqStateText[] =
	{
	"Init",
	"Stopped",
	"Active",
	""
	};


 char *GetMoveSeqStateString(void)
{
	// return point to state descriptor string
	// note that we are not doing any bounds checking..
	return (pstrMoveSeqStateText[eMoveSeqState]);

}


// *************************************************************************************************
//								S y n c h r o n i z a t i o n 
// *************************************************************************************************

// these functions exist to avoid the problem of synchronizing external events with Move Sequences. (specifically serial Menu Commands)
// When a process external to the MoveSequenceFSM (specifically serial Menu Commands) starts a Move Sequence,
// it also needs to know when the Move Sequence is complete. The state of efMoveSequenceEvents is NOT an adequate indicator because
// it the initiating flag is CLEARED as soon as the Move Sequence STARTS, and gives no indication of when it actually completes.
// Overall system timing (see the dispatcher in Main.c) will typically allow the external process to query
// efMoveSequenceEvents before the  Move Sequence has even started to do anyting - and interpret it as  Move Sequence complete.

// so the idea here is that the external process (specifically serial Menu Commands) calls SetMoveSequenceStarted()
// to indicate that a move sequence is in process, setting the fgbIsMoveSequenceStarted flag to TRUE.

// When the MotionPhaseFSM enters ST_CMD_FSM_STOPPED from some other state, the first substate of ST_CMD_FSM_STOPPED calls ClearCommandStarted()
// to clear the fgbIsMotionStarted flag to FALSE.

FILE_GLOBAL_INIT BOOL fgbIsMoveSequenceStarted = FALSE;

void SetMoveSequenceStarted(void)			// always called by higher level, before setting event flag to start Move Sequence
	{
	fgbIsMoveSequenceStarted = TRUE;
	}

void ClearMoveSequenceStarted(void)
	{
	fgbIsMoveSequenceStarted = FALSE;
	}

BOOL IsMoveSequenceComplete(void)
	{
	if (fgbIsMoveSequenceStarted)
		// Command has started, so it cannot be complete
		return FALSE;
	else
		return TRUE;
	}


// *************************************************************************************************
//							M o v e S e q u e n c e T i c k ( ) 
// *************************************************************************************************
// MoveSequenceTick() is called on 100mS timer ticks to decrement the sequence timers

void MoveSequenceTick()
{

	if (fgwTimer1 IS_NOT_ZERO)
		--fgwTimer1;

	if (fgwTimer2 IS_NOT_ZERO)
		--fgwTimer2;

	if (fgwTimer3 IS_NOT_ZERO)
		--fgwTimer3;

	if (fgwCmdTimer IS_NOT_ZERO)	// dedicted SPA command timer
		--fgwCmdTimer;
}


// *************************************************************************************************
//							M o v e S e q u e n c e F S M ( ) 
// *************************************************************************************************
// the MoveSequenceFSM is called on 100mS timer ticks or when there is a non-zero events 
// the MoveSequenceFSM is called when there is a non-zero efMoveSequenceEvents flag


void MoveSequenceFSM(void)
{

	EVENTFLAGS  efMoveSequenceEventsUponEntry;				// used for debugging purposes ONLY, keeps a copy of Motion Event flags at entry to Motion FSM
	EVENTFLAGS  efMoveSequenceEventsUnprocessedThisPass;	// used for debugging purposes ONLY, AND of efMoveSequenceEventsUponEntry and efMoveSequenceEvents at exit from User FSM

	#ifdef __DEBUG
		enum	tagMoveSequenceStepTypes	eStepTypeFoo;
		WORD	nVariableFoo;
		enum	tagMotionPhaseCommands		eCommandFoo;
	#endif	// __DEBUG

	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
        LOCAL ARRAY char strtemp[DISPLAY_LINE_SIZE + 1];
	RTCC_DATE_TIME CurrentDateTime;
	PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
	int nStatus = 0;										// for ftoa()

	efMoveSequenceEventsUponEntry = efMoveSequenceEvents;			// keep a copy of Motion Events upon entry to FSM

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

    switch(eMoveSeqState)
		{

		case ST_MOVE_SEQ_INIT:					// initial state, only at power up
            eMoveSeqState = ST_MOVE_SEQ_STOPPED;
			fSubStateStatus = SUBSTATE_NOT_DONE;
			bMoveSeqFSMSequenceCtr = 0;

			// initialize fgSPAMove structure
			fgSPAMove.fPrevAzimuth = 0.0;
			fgSPAMove.fPrevElevation = 0.0;
			fgSPAMove.fNewAzimuth = 0.0;
			fgSPAMove.fNewElevation = 0.0;
			fgSPAMove.lAzimuthMoveTicks = 0L;
			fgSPAMove.lElevationMoveTicks = 0L;
	        break;


		// *************************************************
		//					STOPPED
		// *************************************************
        case ST_MOVE_SEQ_STOPPED:				// all motion stopped
			{
			// we are STOPPED, so we can process a new move and state change

			// always start with command counter = 0
			fgwCmdExecCounter = 0;

			// cannot exit ST_MOVE_SEQ_STOPPED state until all previous motion has completed
			if((IsCommandComplete(MOTOR_AZIMUTH) IS FALSE) OR (IsCommandComplete(MOTOR_ELEVATION) IS FALSE))			// check for command complete
				{
				// something is still active; cannot do anything
				break;
				}

			// *****************************
			//	 Process new command
			// *****************************
			// this could be reduced to little more than a table of table pointers.. but that will wait until the functionality is stable
			// check for Move Sequence Event Flags, set by MenuFSM to start a move sequence

			// check for Find End Points
			if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_FIND_END_POINTS))
				{
				// clear the calling event flag
				BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_FIND_END_POINTS);

				// no need to initialze move distance

				// clear end of travel values (not really necessary)
				fglLeftEndOfTravel = 0;
				fglRightEndOfTravel = 0;
				fglUpEndOfTravel = 0;
				fglDownEndOfTravel = 0;

				// this may not be necessary; it gives us a mechanical azimuth value of 0 at the current orientation
				CurrentPosition_Clear(MOTOR_AZIMUTH);
				
				// display current orientation (not very meaningful; it should be 0!
				// Find End Points uses Mechanical Orientation
				CurrentMechanicalOrientation_Read(&fgCurrentLocalOrientation);	// read orientation into fgCurrentLocalOrientation structure
				#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                DisplayMessage(SERIAL_MENU_UART,"Initial Mechanical Orientation\r\n", WAIT_FOR_DISPLAY);
				Orientation_Format(szfnDisplayStr, &fgCurrentLocalOrientation);
				DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                #endif
				// initialize sequence step pointer, substate, sequence counter
				fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_FindEndPoints_Sequence[0];
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMoveSeqFSMSequenceCtr = 0;

	            eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
				break;	
				}


			#ifdef USE_ELEVATION
				// check for Move UP (really intended for testing only)
				if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_UP))
					{
					// clear the calling event flag
					BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_UP);

					// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM()
					pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_ELEVATION], MOTOR_ELEVATION);

					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_RunUp_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
					break;
					}

				// check for Move DOWN (really intended for testing only)
				if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_DOWN))
					{
					// clear the calling event flag
					BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_DOWN);

					// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM()
					pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(-pgfMS_StepDistanceDegrees[MOTOR_ELEVATION], MOTOR_ELEVATION);

					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_RunDown_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
					break;
					}
			#endif	// USE_ELEVATION

			#ifdef USE_AZIMUTH
				// check for Move Right (really intended for testing only)
				if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_RIGHT))
					{
					// clear the calling event flag
					BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_RIGHT);

					// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM()
					pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], MOTOR_AZIMUTH);
					#ifdef USE_INCLINOMETER_FEEDBACK
						pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH];
					#endif

					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_RunRight_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
					break;
					}

				// check for Move LEFT (really intended for testing only)
				if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_LEFT))			// set in MenuFSM.c
					{
					// clear the calling event flag
					BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_LEFT);

					// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM()
					pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], MOTOR_AZIMUTH);
					#ifdef USE_INCLINOMETER_FEEDBACK
						pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH];
					#endif

					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_RunLeft_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
					break;
					}

				// check for Incremental Move Right (really intended for testing only)
				if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_INC_RUN_RIGHT))		// set in MenuFSM.c
					{
					// clear the calling event flag
					BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_INC_RUN_RIGHT);

					// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM(), absolute value
					pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], MOTOR_AZIMUTH) / MOVE_INCREMENT_COUNT;
					#ifdef USE_INCLINOMETER_FEEDBACK
						pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] / MOVE_INCREMENT_COUNT;
//						fgfMoveIncrementDegrees[MOTOR_AZIMUTH] = pgfMoveDistanceDegrees[MOTOR_AZIMUTH] / MOVE_INCREMENT_COUNT;
					#endif

					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_IncrementalRunRight_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
					break;
					}

				// check for Incremental Move LEFT (really intended for testing only)
				if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_INC_RUN_LEFT))
					{
					// clear the calling event flag
					BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_INC_RUN_LEFT);

					// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM(), absolute value
					pglMoveDistanceTicks[MOTOR_AZIMUTH] = -ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], MOTOR_AZIMUTH) / MOVE_INCREMENT_COUNT;
					#ifdef USE_INCLINOMETER_FEEDBACK
						pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = -pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] / MOVE_INCREMENT_COUNT;
//						fgfMoveIncrementDegrees[MOTOR_AZIMUTH] = -pgfMoveDistanceDegrees[MOTOR_AZIMUTH] / MOVE_INCREMENT_COUNT;
					#endif

					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_IncrementalRunLeft_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
					break;
					}

			#endif	// USE_AZIMUTH

			// check for SPA Simulation
			if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_SIM))
				{
				// clear the calling event flag
				BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_SIM);

				// initialize the move distance. pgfMS_StepDistanceDegrees[] is set externally by the MenuFSM()
				// this is the size of each 'step' in the MoveSequence. Direction is determined at run time
				pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH], MOTOR_AZIMUTH);
				pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(pgfMS_StepDistanceDegrees[MOTOR_ELEVATION], MOTOR_ELEVATION);
				#ifdef USE_INCLINOMETER_FEEDBACK
					pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH];
				#endif

				// initialize sequence step pointer, substate, sequence counter
				fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_SPASimulation_Sequence[0];
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMoveSeqFSMSequenceCtr = 0;

	            eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
				break;
				}


			// check for SPA Calculate
			if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_CALCULATE))
				{
				// clear the calling event flag
				BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_CALCULATE);

				// initialize sequence step pointer, substate, sequence counter
				fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_SPACalc_Sequence[0];
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMoveSeqFSMSequenceCtr = 0;

	            eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
				break;
				}

			// check for SPA Track
			if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_TRACK))
				{
				// clear the calling event flag
				BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_TRACK);
#ifdef NOT_USED
				if (bFindEndPointsHasRun IS FALSE)
				{
					DisplayMessage(SERIAL_MENU_UART,"Please Run FindEndPoints before SPA Tracking\r\n", WAIT_FOR_DISPLAY);
				}
				else
#endif
				{
					// Find End Points has run, so we can run SPA track
					// initialize sequence step pointer, substate, sequence counter
					fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_SPATrack_Sequence[0];
					fSubStateStatus = SUBSTATE_NOT_DONE;
					bMoveSeqFSMSequenceCtr = 0;

					eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
				}
				break;
				}
                        //check for wind stow/ end wind stow
                        if ((IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))
                                OR(IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_END_WIND_STOW)))
                        {
                            SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
                            BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);	// start with virtual button press for ButtonProcessingFSM() handling
                            BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);		// OPEN will be processed AFTER CLOSED, has the effect of push and release

                                        break;
                        }

			// check for STOP
			if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_STOP))
				{
				// clear the calling event flag
				BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_STOP);

				// initialize sequence step pointer, substate, sequence counter
				fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_Stopped_Sequence[0];
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMoveSeqFSMSequenceCtr = 0;

	            //eMoveSeqState = ST_MOVE_SEQ_STOPPED;				// not much to do here!
	            eMoveSeqState = ST_MOVE_SEQ_ACTIVE;
				break;
				}

			}

			break;						// <sek> 19 Aug 13

			//	EF_MOVE_SEQ_RESET		// reset and restart


		// *************************************************
		//					ACTIVE
		// *************************************************
		// when a move is currently under way, the ONLY command we can process is EF_MOVE_SEQ_STOP
        case ST_MOVE_SEQ_ACTIVE:				// move sequence currently under way

                        if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_WIND_STOW))
                        {
                            BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_WIND_STOW);
                            fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_Stopped_Sequence[0];
                            break;
                        }
                         
			// check for STOP
			if (IS_BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_STOP))
				{
				// clear the calling event flag
				BITCLEAR(efMoveSequenceEvents, EF_MOVE_SEQ_STOP);

				// initialize sequence step pointers
				fgptrMoveSequence = (PTR_MOVE_SEQUENCE_STEP)&fg_Stopped_Sequence[0];

				// stay in ST_MOVE_SEQ_ACTIVE state until Stopped_Sequence is complete
				break;
				}

			// check for state complete
			if (fSubStateStatus IS SUBSTATE_DONE)
				{
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMoveSeqFSMSequenceCtr = 0;
				eMoveSeqState = ST_MOVE_SEQ_STOPPED;			// all done, move to STOPPED
				}

			break;

		}		// end    switch(eMoveSeqState) for state transition


    // ************************************************************************
    //					Process the Current or New State
    // ************************************************************************
    // State handler
    //	called one or more times for a state, depending on state implementation
    //	state transitions do NOT occur here
    //	no fallthrough state transitions - use fFSM_Execute flag to force immediate re-execute for eventless state changes
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate

    switch(eMoveSeqState)
	    {
		case ST_MOVE_SEQ_INIT:
	        RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STATE);		// should never get here!
            break;

		// *************************************************
		//			STOPPED
		// *************************************************
        case ST_MOVE_SEQ_STOPPED:								// all motion stopped
			break;

		// *************************************************
		//			ACTIVE
		// *************************************************
        case ST_MOVE_SEQ_ACTIVE:								// processing an event sequence
			#ifdef __DEBUG
				// these are just for debugging
				eStepTypeFoo = fgptrMoveSequence->eStepType;
				nVariableFoo = fgptrMoveSequence->nVariable;
				eCommandFoo = fgptrMoveSequence->eCommand;
			#endif	// __DEBUG

			switch(fgptrMoveSequence->eStepType)
				{

				// *****************************************
				//			Start Timer
				// *****************************************
				// note use of TIMER_DIVISOR, which allows for easy compile-time shortening of all timers for testing
				case MOVE_SEQ_STEP_START_TIMER1:								// start timer 1
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Start Timer 1\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					fgwTimer1 = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
					++fgptrMoveSequence;										// bump step pointer
					break;

				case MOVE_SEQ_STEP_START_TIMER2:								// start timer 2
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Start Timer 2\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					fgwTimer2 = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
					++fgptrMoveSequence;										// bump step pointer
					break;

				case MOVE_SEQ_STEP_START_TIMER3:								// start timer 3
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Start Timer 3\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					fgwTimer3 = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
					++fgptrMoveSequence;										// bump step pointer
					break;

				// *****************************************
				//			Wait for Timer to Finish
				// *****************************************
				case MOVE_SEQ_STEP_WAIT_TIMER1:					// wait for timer 1 to decrement to 0
					// timers are bumped (decremented) during MoveSequenceFSM_Tick()
					if (fgwTimer1 IS (WORD)0)					// timer finished?
						{
						++fgptrMoveSequence;					// bump step pointer
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"\r\nTimer 1 Done\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
						}
					else
						{
						// timer has not yet finished
						#ifdef USE_MOVE_SEQ_FSM_TIMER_WAIT_VERBOSE
							DisplayStrSequence(SERIAL_MENU_UART, "Wait for Timer 1 ");
						#endif
						}
					break;

				case MOVE_SEQ_STEP_WAIT_TIMER2:					// wait for timer 1 to decrement to 0
					// timers are bumped (decremented) during MoveSequenceFSM_Tick()
					if (fgwTimer2 IS (WORD)0)					// timer finished?
						{
						++fgptrMoveSequence;					// bump step pointer
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"\r\nTimer 2 Done\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
						}
					else
						{
						// timer has not yet finished
						#ifdef USE_MOVE_SEQ_FSM_TIMER_WAIT_VERBOSE
							DisplayStrSequence(SERIAL_MENU_UART, "Wait for Timer 2 ");
						#endif
						}
					break;

				case MOVE_SEQ_STEP_WAIT_TIMER3:					// wait for timer 1 to decrement to 0
					// timers are bumped (decremented) during MoveSequenceFSM_Tick()
					if (fgwTimer2 IS (WORD)0)					// timer finished?
						{
						++fgptrMoveSequence;					// bump step pointer
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"\r\nTimer 3 Done\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
						}
					else
						{
						// timer has not yet finished
						#ifdef USE_MOVE_SEQ_FSM_TIMER_WAIT_VERBOSE
							DisplayStrSequence(SERIAL_MENU_UART, "Wait for Timer 3 ");
						#endif
						}
					break;

				// *****************************************
				//			Start Loop Counter
				// *****************************************
				case MOVE_SEQ_STEP_START_COUNTER1:				// start loop counter 1
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Start Counter 1\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					fgwCounter1 = (WORD)fgptrMoveSequence->nVariable;	// initalize loop counter
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_START_COUNTER2:				// start loop counter 2
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Start Counter 2\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					fgwCounter2 = (WORD)fgptrMoveSequence->nVariable;	// initalize loop counter
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_START_COUNTER3:				// start loop counter 3
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Start Counter 3\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					fgwCounter3 = (WORD)fgptrMoveSequence->nVariable;	// initalize loop counter
					++fgptrMoveSequence;						// bump step pointer
					break;

				// *****************************************
				//	Decrement Loop Counter, Skip on 0
				// *****************************************
				case MOVE_SEQ_STEP_SKIP_ON_COUNTER1:			// decrement and skip next step on counter 1 = 0
					--fgwCounter1;								// decrement counter
					++fgptrMoveSequence;						// bump step pointer

					if (fgwCounter1 IS (WORD)0)					// counter finished?
						{
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"Skip on Counter 1 IS 0\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
						++fgptrMoveSequence;					// bump step pointer to skip next step
						}
					break;

				case MOVE_SEQ_STEP_SKIP_ON_COUNTER2:			// decrement and skip next step on counter 2 = 0
					--fgwCounter2;								// decrement counter
					++fgptrMoveSequence;						// bump step pointer

					if (fgwCounter2 IS (WORD)0)					// counter finished?
						{
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"Skip on Counter 2 IS 0\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
						++fgptrMoveSequence;					// bump step pointer to skip next step
						}
					break;

				case MOVE_SEQ_STEP_SKIP_ON_COUNTER3:			// decrement and skip next step on counter 3 = 0
					--fgwCounter3;								// decrement counter
					++fgptrMoveSequence;						// bump step pointer

					if (fgwCounter3 IS (WORD)0)					// counter finished?
						{
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"Skip on Counter 3 IS 0\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
						++fgptrMoveSequence;					// bump step pointer to skip next step
						}
					break;

				// *****************************************
				//			Execute SPA Command
				// *****************************************
				case MOVE_SEQ_STEP_EXEC_SPA_COMMAND:			// execute command by setting an event flag
					// PanelPositionFSM output is PANEL_POSITION_HOLD, no need to run command
					if (ReadInclinometerSample(&pgInclination) IS FALSE)
					{
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"FAILED Inclinometer\r\n", WAIT_FOR_DISPLAY);
						#endif
						eTrackState = PANEL_POSITION_HOLD;
                                                ++fgptrMoveSequence;					// bump step pointer
						break;
					}

                                        if(Buid_Diff_Date == 1)   // for bosch added for functionality
                                        {
                                            DisplayMessage(SERIAL_MENU_UART,"\nBeyond limited days\r\n", WAIT_FOR_DISPLAY);
                                            eTrackState = PANEL_POSITION_HOLD;
                                            ++fgptrMoveSequence;					// bump step pointer
						break;
                                        }
                                        else
                                        {
                                             DisplayMessage(SERIAL_MENU_UART,"\nin limited days\r\n", WAIT_FOR_DISPLAY);
                                        }
					// requested move is long enough to execute
					#ifdef USE_AZIMUTH
						if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
						{
							// check for short Azimuth move distance
							if (ABS(fgSPAMove.lAzimuthMoveTicks) < RUN_MINIMUM_MOVE_TICKS(fgptrMoveSequence->eAxis))
							{
								eTrackState = PANEL_POSITION_HOLD;			// treat as HOLD, even if PanelPositionFSM() returned MOVE
								#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									DisplayMessage(SERIAL_MENU_UART,"Short Azimuth Move, No Need to Execute Command\r\n", WAIT_FOR_DISPLAY);
								#endif
								++fgptrMoveSequence;					// bump step pointer
								break;									// done with SPA command processing
							}
                                                        eTrackState = PANEL_POSITION_MOVE;
							// if the command is an unspecified Run, check for direction
							if (fgptrMoveSequence->eCommand IS EF_MTN_CMD_RUN)			// this is typical of SPA tracking
							{
								// NOTE: we do not process PWM_DIR_STOPPED here, nothing to do!
								if (fgSPAMove.eAzimuthDirection IS PWM_DIR_FORWARD)
								{
									// Run forward
                                                                        bTrackerDirection = 2;
									BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_FWD);
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Start Cmd Timer\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									fgwCmdTimer = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
									SetCommandStarted(fgptrMoveSequence->eAxis);				// mark command as started so we cannot misinterpret completion

								}
								else if (fgSPAMove.eAzimuthDirection IS PWM_DIR_REVERSE)
								{
									// Run reverse
                                                                        bTrackerDirection = 1;
									BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_REV);
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Start Cmd Timer\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									fgwCmdTimer = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
									SetCommandStarted(fgptrMoveSequence->eAxis);				// mark command as started so we cannot misinterpret completion
								}
								else if (fgSPAMove.eAzimuthDirection IS PWM_DIR_STOPPED)
								{
									// Stopped (huh?)
									BITCLEAR(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_FWD);
									BITCLEAR(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_REV);
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Azimuth Move STOPPED\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
								}
							}
							else
							{
								// not an unspecified command, use as is
								BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], fgptrMoveSequence->eCommand);
								#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									DisplayMessage(SERIAL_MENU_UART,"Start Cmd Timer\r\n", NO_WAIT_FOR_DISPLAY);
								#endif
								fgwCmdTimer = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
								SetCommandStarted(fgptrMoveSequence->eAxis);				// mark command as started so we cannot misinterpret completion
							}

						}		//  (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
					#endif	// USE_AZIMUTH

					#if defined(USE_AZIMUTH) AND defined (USE_ELEVATION)
						else
					#endif	// defined(USE_AZIMUTH) AND defined (USE_ELEVATION)

					#ifdef USE_ELEVATION
						if (fgptrMoveSequence->eAxis IS AXIS_ELEVATION)
						{
							// check for 0 to 4 tick Elevation move
							if (ABS(fgSPAMove.lElevationMoveTicks) < RUN_MINIMUM_MOVE_TICKS(fgptrMoveSequence->eAxis))
							{
								eMoveState = PANEL_POSITION_HOLD;			// treat as HOLD, even if PanelPositionFSM() returned MOVE
								#ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
									DisplayMessage(SERIAL_MENU_UART,"Short Elevation Move, No Need to Execute Command\r\n", WAIT_FOR_DISPLAY);
								#endif
								++fgptrMoveSequence;					// bump step pointer
								break;
							}

							// if the command is an unspecified Run, check for direction
							if (fgptrMoveSequence->eCommand IS EF_MTN_CMD_RUN)			// this is typical of SPA tracking
							{
								// NOTE: we do not process PWM_DIR_STOPPED here, nothing to do!
								if (fgSPAMove.eElevationDirection IS PWM_DIR_FORWARD)
								{
									// Run forward
									BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_FWD);
									#ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
										DisplayMessage(SERIAL_MENU_UART,"Start Cmd Timer\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									fgwCmdTimer = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
									SetCommandStarted(fgptrMoveSequence->eAxis);				// mark command as started so we cannot misinterpret completion
								}
								else if (fgSPAMove.eElevationDirection IS PWM_DIR_REVERSE)
								{
									// Run reverse
									BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_REV);
									#ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
										DisplayMessage(SERIAL_MENU_UART,"Start Cmd Timer\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									fgwCmdTimer = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
									SetCommandStarted(fgptrMoveSequence->eAxis);				// mark command as started so we cannot misinterpret completion
								}
								else if (fgSPAMove.eElevationDirection IS PWM_DIR_STOPPED)
								{
									// Stopped (huh?)
									BITCLEAR(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_FWD);
									BITCLEAR(efMotionPhaseCommands[fgptrMoveSequence->eAxis], EF_MTN_CMD_RUN_REV);
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Elevation Move STOPPED\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
								}
							}
							else
							{
								// not an unspecified command, use as is
								BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], fgptrMoveSequence->eCommand);
								#ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
									DisplayMessage(SERIAL_MENU_UART,"Start Cmd Timer\r\n", NO_WAIT_FOR_DISPLAY);
								#endif
								fgwCmdTimer = (WORD)fgptrMoveSequence->nVariable / TIMER_DIVISOR;	// initalize countdown timer
								SetCommandStarted(fgptrMoveSequence->eAxis);				// mark command as started so we cannot misinterpret completion
							}

						}		//  (fgptrMoveSequence->eAxis IS AXIS_ELEVATION)
					#endif	// USE_ELEVATION

					++fgwCmdExecCounter;						// bump command execute counter

					++fgptrMoveSequence;						// bump step pointer
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"SPA Execute Command\r\n", NO_WAIT_FOR_DISPLAY);
					#endif

#ifdef NOT_USED
					{
						DisplayMessage(SERIAL_MENU_UART,"Not a Valid Command\r\n", WAIT_FOR_DISPLAY);
						RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STEP);
					}
#endif

					break;

				// *****************************************
				//			Wait SPA Command
				// *****************************************
				case MOVE_SEQ_STEP_WAIT_SPA_COMMAND:				// wait for command complete
					if ((eMoveState IS PANEL_POSITION_HOLD) AND (eTrackState IS PANEL_POSITION_HOLD))
					{
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"No Need to wait for Command\r\n", WAIT_FOR_DISPLAY);
						#endif
						++fgptrMoveSequence;					// bump step pointer
						break;
					}
                                        
					// check for completion of previously selected command
					if (IsCommandComplete(fgptrMoveSequence->eAxis) IS_TRUE)			// check for command complete
					{
						// write entire NV Parameters table from MCU RAM to RTCC NV RAM
//						WriteRTCCRAMParameterTable();

						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"\r\nCommand Done", WAIT_FOR_DISPLAY);
						#endif
						fgwCmdTimer = 0;							// stop command timer
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						AddDisplayStr("Cur Orientation (degreesWS): ");
                                                #endif
						CurrentLocalOrientation_Format(szfnDisplayStr);		// get current orientation, formatted for output
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                                AddDisplayStrAndNewLine(szfnDisplayStr);
						DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                                #endif
						#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
							// save time, Azimuth at end of move and backtracking angles in Polar Axis Move Table (for later display)
							if (ReadRTCCDateTime(ptrDateTime) IS_NOT TRUE)
							{
                                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
								DisplayMessage(SERIAL_MENU_UART,"Failed Read RTCC\r\n", WAIT_FOR_DISPLAY);
                                                                #endif
								pgPolarAxisMoveTbl[pgbPolarAxisMoveTblIndex].cHours = 0;
							}
							else
							{
								pgPolarAxisMoveTbl[pgbPolarAxisMoveTblIndex].cHours = ptrDateTime->cHours;
							}
							pgPolarAxisMoveTbl[pgbPolarAxisMoveTblIndex].fAzimuthAngleDegrees = CurrentAverageAngleDegrees_Read(MOTOR_AZIMUTH);
							pgPolarAxisMoveTbl[pgbPolarAxisMoveTblIndex].fModuleTiltAngleDegrees = fgSPAOrientation.fModuleTiltAngleDegrees;
							pgPolarAxisMoveTbl[pgbPolarAxisMoveTblIndex].fBackTrackAngleDegrees = fgSPAOrientation.fBackTrack;
							pgPolarAxisMoveTbl[pgbPolarAxisMoveTblIndex].fSunElevationAngleDegrees = fgSPAOrientation.fSunElevationAngleDegrees;

							++pgbPolarAxisMoveTblIndex;					// bump move table index

							// check for table index rollover
							if (pgbPolarAxisMoveTblIndex >= POLAR_AXIS_MOVE_TBL_LEN)
							{
								pgbPolarAxisMoveTblIndex = 0;				// restart/wrap index
							}

						#endif	// defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)

						// display movement error stats
						// move error in degrees is the current orientation (converted from ticks to degrees) - requested orientation in degrees
						// the conversion from current orientation in ticks to degrees may introduce some math error, and because the numbers
						// involved are much larger than the move error, and the errors are accumulated, so the error will be larger..
						CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						ClearDisplayStr();
						AddDisplayStr("\r\nCur Orientation (ticks):\t");
						INT32StoASCIIstr(fgCurrentLocalOrientation.lAzimuthPositionTicks, INT32S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);
						AddDisplayStr(", ");
						INT32StoASCIIstr(fgCurrentLocalOrientation.lElevationPositionTicks, INT32S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);
						AddDisplayNewLine();						// add line terminator
						DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                                #endif
						// move error in ticks is actual move - requested move
						fgSPAMove.lAzimuthMoveErrorTicks = (INT32S)(pgMotionStats[MOTOR_AZIMUTH].ulMSI_TotalCount - (UINT32)ABS(fgSPAMove.lAzimuthMoveTicks));
						fgSPAMove.lElevationMoveErrorTicks = (INT32S)(pgMotionStats[MOTOR_ELEVATION].ulMSI_TotalCount - (UINT32)ABS(fgSPAMove.lElevationMoveTicks));
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						ClearDisplayStr();
						AddDisplayStr("Move Errors (ticks):\t\t");
						INT32StoASCIIstr(fgSPAMove.lAzimuthMoveErrorTicks, INT32S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);
						AddDisplayStr(", ");
						INT32StoASCIIstr(fgSPAMove.lElevationMoveErrorTicks, INT32S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);
						AddDisplayNewLine();						// add line terminator
						DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                                #endif
						fgSPAMove.fAzimuthPositionErrorDegrees = fgSPAMove.fNewAzimuth - fgCurrentLocalOrientation.fAzimuth;
						fgSPAMove.fElevationPositionErrorDegrees = fgSPAMove.fNewElevation - fgCurrentLocalOrientation.fElevation;
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						AddDisplayStr("\r\nAccumulated Position Errors (degrees): ");
						AddDisplayStr((const char *)ftoa(fgSPAMove.fAzimuthPositionErrorDegrees, &nStatus));
						AddDisplayStr(", ");
						AddDisplayStr((const char *)ftoa(fgSPAMove.fElevationPositionErrorDegrees, &nStatus));
						AddDisplayNewLine();						// add line terminator
						DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                                #endif
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							AddDisplayStr("\r\nCommand Done: Total Command Count: ");
							WORDtoASCIIstr(fgwCmdExecCounter, INT16S_WIDTH, szfnDisplayStr);
							AddDisplayStr(szfnDisplayStr);
							AddDisplayNewLine();					// add line terminator
							DisplayStrWait(SERIAL_MENU_UART);		// start display (serial output) of line
						#endif

						++fgptrMoveSequence;						// bump step pointer
					}
					else
					{
						// command has not yet completed

						// check for timeout
						// timers are bumped (decremented) during MoveSequenceFSM_Tick()
						if (fgwCmdTimer IS (WORD)0)					// timer finished?
						{
							// terminate executing command, so that the next pass through MoveSequenceFSM() will process command complete
							ClearCommandStarted((enum tagMotors) fgptrMoveSequence->eAxis);
							#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                                        DisplayMessage(SERIAL_MENU_UART,"\r\nSPA Command Timer TIMEOUT\r\n", WAIT_FOR_DISPLAY);
                                                        #endif
							Finish_MotionStats((enum tagMotors) fgptrMoveSequence->eAxis);
							// ==>> does the previous command ever get completed?
							SetCommandStarted((enum tagMotors) fgptrMoveSequence->eAxis);								// mark command as started so we cannot misinterpret completion
							BITSET(efMotionPhaseCommands[(enum tagMotors) fgptrMoveSequence->eAxis], EF_MTN_CMD_STOP);	// bring to an orderly stop
							ResetMotionFSM();												// prepare MotionFSM to resetart
						}
						
                                                else
                                                {
                                                    /*if((fgdefferenceAngle = fabs(CurrentAverageAngleDegrees_Read(MOTOR_AZIMUTH)-(ptrRTCC_RAM_AppParameters->fSetPoint_AZ))) <= 0.2)
                                                    {
                                                        ClearCommandStarted((enum tagMotors) fgptrMoveSequence->eAxis);
							#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                                        DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
                                                        #endif
							Finish_MotionStats((enum tagMotors) fgptrMoveSequence->eAxis);
							// ==>> does the previous command ever get completed?
							SetCommandStarted((enum tagMotors) fgptrMoveSequence->eAxis);								// mark command as started so we cannot misinterpret completion
							BITSET(efMotionPhaseCommands[(enum tagMotors) fgptrMoveSequence->eAxis], EF_MTN_CMD_STOP);	// bring to an orderly stop
							ResetMotionFSM();	
                                                    }*/
                                                    #ifdef USE_MOVE_SEQ_FSM_WAIT_VERBOSE
                                                        // show current orientation as command continues
                                                        strcpy(szfnDisplayStr, "Current");
                                                        CurrentLocalOrientation_Format(szfnDisplayStr + strlen(szfnDisplayStr));		// get current orientation, formatted for output
                                                        strcat(szfnDisplayStr, " Wait for Command ");
                                                        WORDtoASCIIstr((WORD)fgwCmdTimer, WORD_WIDTH, strtemp);
                                                        strcat(szfnDisplayStr,strtemp);
                                                       // strcat(szfnDisplayStr, "  diff: ");
                                                     //   strcat(szfnDisplayStr, (const char *)ftoa(fgdefferenceAngle, (int *)&nStatus));
                                                        DisplayStrSequence(SERIAL_MENU_UART, szfnDisplayStr);
                                                    #endif
                                                }
						
					}
					break;


				// *****************************************
				//			Execute Run or Slew Command
				// *****************************************
				case MOVE_SEQ_STEP_EXEC_COMMAND:				// execute command by setting an event flag
					// start command
					if ((fgptrMoveSequence->eAxis IS AXIS_AZIMUTH) OR (fgptrMoveSequence->eAxis IS AXIS_ELEVATION))
					{
						SetCommandStarted(fgptrMoveSequence->eAxis);// mark command as started so we cannot misinterpret completion

						BITSET(efMotionPhaseCommands[fgptrMoveSequence->eAxis], fgptrMoveSequence->eCommand);

						++fgptrMoveSequence;						// bump step pointer
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"Execute Command\r\n", NO_WAIT_FOR_DISPLAY);
						#endif
					}
					else
					{
                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Not a Valid Command\r\n", WAIT_FOR_DISPLAY);
                                                #endif
						RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STEP);
					}
					break;

				case MOVE_SEQ_STEP_WAIT_COMMAND:				// wait for command complete
					// check for completion of previously selected command
					if (IsCommandComplete(fgptrMoveSequence->eAxis) IS_TRUE)			// check for command complete
						{
                                                CurrentLocalOrientation_Format(szfnDisplayStr);		// get current orientation, formatted for output
                                                MAN_WEST = MAN_EAST = MAN_STOW = 0;
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"\r\nCommand Done", WAIT_FOR_DISPLAY);
            					AddDisplayStr("Cur Orientation (degreesSTW): ");
						AddDisplayStrAndNewLine(szfnDisplayStr);
						DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                                #endif
						++fgptrMoveSequence;					// bump step pointer
						}
					else
						{
						// command has not yet completed
						#ifdef USE_MOVE_SEQ_FSM_WAIT_VERBOSE
							// show current orientation as command continues
							strcpy(szfnDisplayStr, "Current");
							CurrentLocalOrientation_Format(szfnDisplayStr + strlen(szfnDisplayStr));		// get current orientation, formatted for output
							strcat(szfnDisplayStr, " Wait for Command ");
							DisplayStrSequence(SERIAL_MENU_UART, szfnDisplayStr);
						#endif
						}
					break;

				// *****************************************
				//				Labels
				// *****************************************
				// labels are just placeholders
				case MOVE_SEQ_STEP_LABEL1:						// loop label 1
					++fgptrMoveSequence;						// bump step pointer
					fgptrLabel1 = fgptrMoveSequence;			// save pointer to step after Label 1, this is the actual loop point
					break;

				case MOVE_SEQ_STEP_LABEL2:						// loop label 2
					++fgptrMoveSequence;						// bump step pointer
					fgptrLabel2 = fgptrMoveSequence;			// save pointer to step after Label 2, this is the actual loop point
					break;

				case MOVE_SEQ_STEP_LABEL3:						// loop label 3
					++fgptrMoveSequence;						// bump step pointer
					fgptrLabel3 = fgptrMoveSequence;			// save pointer to step after Label 3, this is the actual loop point
					break;

				// *****************************************
				//				Goto Labels
				// *****************************************
				// ==> these should check for valid label pointer
				case MOVE_SEQ_STEP_GOTO_LABEL1:					// jump to loop label 1
					fgptrMoveSequence = fgptrLabel1;
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Goto Label 1\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					break;

				case MOVE_SEQ_STEP_GOTO_LABEL2:					// jump to loop label 2
					fgptrMoveSequence = fgptrLabel2;
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Goto Label 2\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					break;

				case MOVE_SEQ_STEP_GOTO_LABEL3:					// jump to loop label 3
					fgptrMoveSequence = fgptrLabel3;
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Goto Label 3\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					break;

				// *****************************************
				//		Save/Restore Orientation
				// *****************************************
				case MOVE_SEQ_STEP_SAVE_ORIENTATION1:			// save current orientation to fgOrientation1
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						strcpy(szfnDisplayStr,"Save Orientation1\r\n");
					#endif
					CurrentLocalOrientation_Read(&fgOrientation1);	// read Local Orientation, adjusted by offset, into fgOrientation1 structure
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgOrientation1);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_SAVE_ORIENTATION2:			// save current orientation to fgOrientation2
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						strcpy(szfnDisplayStr,"Save Orientation2\r\n");
					#endif
					CurrentLocalOrientation_Read(&fgOrientation2);	// read orientation
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgOrientation2);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_SAVE_ORIENTATION3:			// save current orientation to fgOrientation3
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						strcpy(szfnDisplayStr,"Save Orientation3\r\n");
					#endif
					CurrentLocalOrientation_Read(&fgOrientation3);	// read orientation
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgOrientation3);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_RESTORE_ORIENTATION1:		// set destination to fgOrientation1
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						strcpy(szfnDisplayStr,"Restore Orientation1, Current: \r\n");
					#endif
					CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);	// read current orientation
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgCurrentLocalOrientation);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif

					// set move distance to restore orientation
					// ==> need for bounds check here?
					pglMoveDistanceTicks[MOTOR_AZIMUTH] = ABS(fgCurrentLocalOrientation.lAzimuthPositionTicks - fgOrientation1.lAzimuthPositionTicks);
					pglMoveDistanceTicks[MOTOR_ELEVATION] = ABS(fgCurrentLocalOrientation.lElevationPositionTicks - fgOrientation1.lElevationPositionTicks);
					// ==>> this should not need a value in degrees, because it is not a move?
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_RESTORE_ORIENTATION2:		// set destination to fgOrientation2
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						strcpy(szfnDisplayStr,"Restore Orientation2, Current: \r\n");
					#endif
					CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);	// read current orientation
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgCurrentLocalOrientation);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					// set move distance to restore orientation
					// ==> need for bounds check here?
					pglMoveDistanceTicks[MOTOR_AZIMUTH] = ABS(fgCurrentLocalOrientation.lAzimuthPositionTicks - fgOrientation2.lAzimuthPositionTicks);
					pglMoveDistanceTicks[MOTOR_ELEVATION] = ABS(fgCurrentLocalOrientation.lElevationPositionTicks - fgOrientation2.lElevationPositionTicks);
					// ==>> this should not need a value in degrees, because it is not a move?
					++fgptrMoveSequence;						// bump step pointer
					break;

				case MOVE_SEQ_STEP_RESTORE_ORIENTATION3:		// set destination to fgOrientation3
					// ==>> this should not need a value in degrees, because it is not a move?
					++fgptrMoveSequence;						// bump step pointer
					break;

				// *****************************************
				//	Set Distance to Move (single axis)
				// *****************************************
				// testing AND move-to-stow

				case MOVE_SEQ_STEP_SET_MOVE_DISTANCE:			// set distance to move
					if ((fgptrMoveSequence->eAxis IS AXIS_AZIMUTH) OR (fgptrMoveSequence->eAxis IS AXIS_ELEVATION))
					{
						pglMoveDistanceTicks[fgptrMoveSequence->eAxis] = (INT32)fgptrMoveSequence->nVariable;
						#ifdef USE_INCLINOMETER_FEEDBACK
							if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
							{
								pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = ((float)fgptrMoveSequence->nVariable / AZ_FLOAT_MSI_TICKS_PER_DEGREE);
							}
						#endif

					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Set Distance\r\n", NO_WAIT_FOR_DISPLAY);
					#endif
					}
					else
					{
                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Not a Valid Command\r\n", WAIT_FOR_DISPLAY);
                                                #endif
						RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STEP);
					}
					++fgptrMoveSequence;						// bump step pointer
					break;


				// *****************************************
				//		Calculate New Center
				// *****************************************
				case MOVE_SEQ_STEP_CALC_CENTER:

					#ifdef USE_AZIMUTH
						if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
						{
							// recover Azimuth positions at End-of-Travel
							// these positions will be relative to the previous Mechanical Center Orientation
							fglRightEndOfTravel = fgOrientation1.lAzimuthPositionTicks;		// most positive position (or least negative position)
							fglLeftEndOfTravel = fgOrientation2.lAzimuthPositionTicks;		// most negative position (or least positive position)

							// calculate total travel distance between end stops/limit switches
							// this calculation is setup to handle all possibilities,
							// including some which can only occur during development, with no mechanical movement restrictions.
							if ((fglRightEndOfTravel > 0) AND (fglLeftEndOfTravel > 0))
								{
								// both end points are positive, so we are WAY to the Right..
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(fglRightEndOfTravel - fglLeftEndOfTravel);
								}
							else if ((fglRightEndOfTravel < 0) AND (fglLeftEndOfTravel < 0))
								{
								// both end points are negative, so we are WAY to the left (notice that the equation is reversed!)
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(ABS(fglLeftEndOfTravel) - ABS(fglRightEndOfTravel));
								}
							else if ((fglRightEndOfTravel > 0) AND (fglLeftEndOfTravel < 0))
								{
								// this is the 'normal' case, right end positive and left end negative
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(fglRightEndOfTravel + ABS(fglLeftEndOfTravel));
								}
							else
								{
								// one of the values must be 0!
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(ABS(fglRightEndOfTravel) + ABS(fglLeftEndOfTravel));
								}


							// we are currently at REVERSE (to left) end of travel
							// calculate new current position relatve to new center
							CurrentPosition_Set(fgptrMoveSequence->eAxis, -(INT32)(fgulTotalTravel[fgptrMoveSequence->eAxis] / 2));

							// the fglXxxxEndOfTravel values are no longer correct, because we have changed the center location
							// update soft limits based on +-(TotalTravel/2)
							ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse = -((float)(fgulTotalTravel[fgptrMoveSequence->eAxis] / 2)) / AZ_FLOAT_MSI_TICKS_PER_DEGREE;
							ptrRAM_SystemParameters->fAZ_SoftLimit_Forward = ((float)(fgulTotalTravel[fgptrMoveSequence->eAxis] / 2)) / AZ_FLOAT_MSI_TICKS_PER_DEGREE;
						}
					#endif	// USE_AZIMUTH

					#ifdef USE_ELEVATION
						if (fgptrMoveSequence->eAxis IS AXIS_ELEVATION)
						{
							// recover Azimuth positions at End-of-Travel
							// these positions will be relative to the previous Mechanical Center Orientation
							fglUpEndOfTravel = fgOrientation1.lElevationPositionTicks;		// most positive position (or least negative position)
							fglDownEndOfTravel = fgOrientation2.lElevationPositionTicks;		// most negative position (or least positive position)

							// calculate total travel distance between end stops/limit switches
							// this calculation is setup to handle all possibilities,
							// including some which can only occur during development, with no mechanical movement restrictions.
							if ((fglUpEndOfTravel > 0) AND (fglDownEndOfTravel > 0))
								{
								// both end points are positive, so we are WAY Up..
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(fglUpEndOfTravel - fglDownEndOfTravel);
								}
							else if ((fglUpEndOfTravel < 0) AND (fglDownEndOfTravel < 0))
								{
								// both end points are negative, so we are WAY down (basically impossible, but notice that the equation is reversed!)
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(ABS(fglDownEndOfTravel) - ABS(fglUpEndOfTravel));
								}
							else if ((fglUpEndOfTravel > 0) AND (fglDownEndOfTravel < 0))
								{
								// this is the 'normal' case, up end positive and down end negative
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(fglUpEndOfTravel + ABS(fglDownEndOfTravel));
								}
							else
								{
								// one of the values must be 0!
								fgulTotalTravel[fgptrMoveSequence->eAxis] = (UINT32)(ABS(fglUpEndOfTravel) + ABS(fglDownEndOfTravel));
								}

							// the fglXxxxEndOfTravel values are no longer correct, because we have changed the center location
							// update soft limits based on Night Stow position (fixed) and TotalTravel
							ptrRAM_SystemParameters->fEL_SoftLimit_Reverse = EL_NIGHT_STOW_DEGREES;
							// this calculation is dependent on EL_NIGHT_STOW_DEGREES being negative
							ptrRAM_SystemParameters->fEL_SoftLimit_Forward = (((float)(fgulTotalTravel[fgptrMoveSequence->eAxis])) / EL_FLOAT_MSI_TICKS_PER_DEGREE) + EL_NIGHT_STOW_DEGREES;

							// we are currently at REVERSE (Down) end of travel
							// calculate new current position, which is the Night Stow position
							CurrentPosition_Set(fgptrMoveSequence->eAxis, (INT32)(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse + CAST_FLOAT_ROUNDING_OFFSET));

						}
					#endif	// USE_ELEVATION

					
					// write the updated parameter table to SPI Flash memory
					WriteFlashParameterTable();		// ==> should have a return value

					#if defined(USE_AZIMUTH) AND defined(USE_ELEVATION)
						// removed for single axis testing, which does NOT move to the center. Perhaps make this a separate command?
						// determine which side of (the newly set) center we are..
						// to determine the move to the new CENTER, which is the Mechanical Center
						if (CurrentPosition_Read(fgptrMoveSequence->eAxis) < 0)
							{
							// we are to the left of center (actually at the left end of travel), so we need to move right (FORWARD)
							pglMoveDistanceTicks[fgptrMoveSequence->eAxis] = (UINT32)ABS(CurrentPosition_Read(fgptrMoveSequence->eAxis));
							#ifdef USE_INCLINOMETER_FEEDBACK
								if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
								{
									pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = CurrentAverageAngleDegrees_Read(fgptrMoveSequence->eAxis);
								}
							#endif
							}
						else if (CurrentPosition_Read(fgptrMoveSequence->eAxis) > 0)
							{
							// we are to the right/above of (the newly set) center, and are therefore seriously LOST
							RuntimeError(MOVE_SEQ_FSM_ERROR_CANNOT_FIND_CENTER);
							AddDisplayStrAndNewLine("Cannot Find Center");
							DisplayStr(SERIAL_MENU_UART);				// start display (serial output) of line
							}
						else
							{
							// we are AT the center, and do not need to do anything! (highly unlikely, and probably also meaning we are LOST!)
							pglMoveDistanceTicks[fgptrMoveSequence->eAxis] = 0;
							#ifdef USE_INCLINOMETER_FEEDBACK
								if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
								{
									pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = 0.0;
								}
							#endif
							#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
								AddDisplayStrAndNewLine("At Center");
								DisplayStr(SERIAL_MENU_UART);				// start display (serial output) of line
							#endif
							}
					#endif

					// we have established new values for CurrentPosition, which are relative to a new center at 0,0
					// reset the value of Orientation1, so when we move from the (new) current position to the (new) Orientation1
					// we will be at the center of movement.
					// Azimuth center is 0.0
					fgOrientation1.fAzimuth = 0.0;					// Float Azimuth. Units: degrees
					fgOrientation1.lAzimuthPositionTicks = 0L;		// Azimuth, Units: MSI ticks
					// Elevation center is center of travel. Note calculation is dependent on fEL_SoftLimit_Reverse < 0
					fgOrientation1.fElevation = (ptrRAM_SystemParameters->fEL_SoftLimit_Forward + ptrRAM_SystemParameters->fEL_SoftLimit_Reverse) / 2.0;	// Float Elevation. Units: degrees
					fgOrientation1.lElevationPositionTicks = ConvertDegreesToMSITicks(fgOrientation1.fElevation, AXIS_ELEVATION);	// Elevation, Units: MSI ticks

//					bFindEndPointsHasRun = TRUE;						// mark FindEndPoints has run

					++fgptrMoveSequence;							// bump step pointer
					break;


				// *****************************************
				//		Restore Saved Sun Position
				// *****************************************
				// this is used ONLY to restore the saved last Sun Position after powerup

				case MOVE_SEQ_RESTORE_SUN_POSITION:
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Restore Previously Stored SPAOrientation\r\n", WAIT_FOR_DISPLAY);
					#endif

					// read current orientation and convert to degrees
					// previous values should have already been restored by 1st call to PanelPositionFSM()
					CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);
					fgSPAMove.fNewAzimuth = fgCurrentLocalOrientation.fAzimuth;
					fgSPAMove.fNewElevation = fgCurrentLocalOrientation.fElevation;
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					strcpy(szfnDisplayStr, "Prev Stored ");
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgCurrentLocalOrientation);
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif

					++fgptrMoveSequence;							// bump step pointer
					break;

				// *****************************************
				//		Calculate Sun Position
				// *****************************************
				case MOVE_SEQ_STEP_CALC_SUN_POSITION:				// save current sun position to fgSPAOrientation
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Calculate Sun Position to SPAOrientation\r\n", WAIT_FOR_DISPLAY);
					#endif

					// get, display current date and time
					if (ReadRTCCDateTime(ptrDateTime) IS_NOT TRUE)
					{
                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Failed Read RTCC\r\n", WAIT_FOR_DISPLAY);
                                                #endif
					}
					else
					{
                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, ptrDateTime);
						DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                            #endif
                                            Buid_Diff_Date = GETDIFF(szfnDisplayStr, ptrDateTime);
					}

					// save current mechanical orientation in RAM copy of RTCC NV RAM
					ptrRTCC_RAM_AppParameters->lAzimuthAtSPA =  CurrentPosition_Read(MOTOR_AZIMUTH);
					ptrRTCC_RAM_AppParameters->lElevationAtSPA =  CurrentPosition_Read(MOTOR_ELEVATION);

					// calculate and display Sun Position, Global SPA Coordinates
					/*fgSPAMove.SunPositionState = */
                                        IGNORE_RETURN_VALUE CalculateSunPosition(&fgSPAOrientation, ptrDateTime);
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                        strcpy(szfnDisplayStr, "SPA\t");
					SPA_Backtrack_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					// save SPA calculation values in RAM copy of RTCC NV RAM
					ptrRTCC_RAM_AppParameters->fSPACalculation_AZ = fgSPAOrientation.fAzimuth;
					ptrRTCC_RAM_AppParameters->fSPACalculation_EL = fgSPAOrientation.fElevation;

                                        // convert SPA orientation to Local Orientation
					ConvertSPAtoLocalOrientation(&fgSPAOrientation);
					// display SPA Local orientation
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					strcpy(szfnDisplayStr, "SPA Local ");
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					// display orientation adjusted for Backtracking
					IGNORE_RETURN_VALUE AdjustForBacktracking(&fgSPAOrientation);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					strcpy(szfnDisplayStr, "Backtrack ");
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					// bounds check and adjust Orientation for limits or stow positions (based on local coordinates, NOT SPA coordinates)
					eMoveState = PanelPositionFSM(&fgSPAOrientation);
                                         #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					strcpy(szfnDisplayStr, "SetPt\t");
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                        #endif
					// save Set Point values in RAM copy of RTCC NV RAM
					ptrRTCC_RAM_AppParameters->fSetPoint_AZ = fgSPAOrientation.fAzimuth;
					ptrRTCC_RAM_AppParameters->fSetPoint_EL = fgSPAOrientation.fElevation;

////					WriteRTCCRAMParameterTable();								// write updated RAM copy of RTCC NV RAM to RTCC

					// if this is the FIRST move after powerup, the previous orientation is the values stored in RTCC flash

					// update fgSPAMove structure with new (destination) orientation
					fgSPAMove.fPrevAzimuth = fgSPAMove.fNewAzimuth;				// note that this is previous SPA Calculated Orientation, NOT the actual orientation
					fgSPAMove.fPrevElevation = fgSPAMove.fNewElevation;
					fgSPAMove.fNewAzimuth = fgSPAOrientation.fAzimuth;
					fgSPAMove.fNewElevation = fgSPAOrientation.fElevation;

					// get current Local Orientation, adjusted for offset from Mechanical Orientation
					CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					AddDisplayStr("\r\nCur Orientation (ticks):      ");
					INT32StoASCIIstr(fgCurrentLocalOrientation.lAzimuthPositionTicks, INT32S_WIDTH, szfnDisplayStr);
					AddDisplayStr(szfnDisplayStr);
					AddDisplayStr(", ");
					INT32StoASCIIstr(fgCurrentLocalOrientation.lElevationPositionTicks, INT32S_WIDTH, szfnDisplayStr);
					AddDisplayStr(szfnDisplayStr);
					AddDisplayNewLine();						// add line terminator
					DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                        #endif
					// convert Movement Degrees to Ticks, and determine direction of movement
					ConvertSPAMoveDegreesToTicks(&fgSPAMove, &fgCurrentLocalOrientation);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					ClearDisplayStr();
					AddDisplayStr("Prev SPA Orientation (ticks): ");
					INT32StoASCIIstr(fgSPAMove.lPrevAzimuthTicks, INT32S_WIDTH, szfnDisplayStr);
					AddDisplayStr(szfnDisplayStr);
					AddDisplayStr(", ");
					INT32StoASCIIstr(fgSPAMove.lPrevElevationTicks, INT32S_WIDTH, szfnDisplayStr);
					AddDisplayStr(szfnDisplayStr);
					AddDisplayNewLine();						// add line terminator
					DisplayStrWait(SERIAL_MENU_UART);				// start display (serial output) of line
                                        
					// display SPA move for both axis
					// more for each axis will be displayed again at the beginning of the move
					AddDisplayStr("SPA Move (ticks):             ");
					INT32StoASCIIstr(fgSPAMove.lAzimuthMoveTicks, INT32S_WIDTH, szfnDisplayStr);
					AddDisplayStr(szfnDisplayStr);
					AddDisplayStr(", ");
					INT32StoASCIIstr(fgSPAMove.lElevationMoveTicks, INT32S_WIDTH, szfnDisplayStr);
					AddDisplayStr(szfnDisplayStr);
					AddDisplayNewLine();						// add line terminator
					DisplayStrWait(SERIAL_MENU_UART);				// start display (serial output) of line
                                        #endif
					++fgptrMoveSequence;						// bump step pointer
					break;


				// *****************************************
				//		Simulator Initialization
				// *****************************************
				case MOVE_SEQ_SIM_INIT:
					// initialize simulation DateTime to program start time
					if (ReadRTCCDateTime(&SimDateTime) IS_NOT TRUE)
					{
                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Unable to Read RTCC\r\n", WAIT_FOR_DISPLAY);
                                                #endif
					}
					else
					{
						// Adjust time to 12:01 AM
						SimDateTime.cHours = 6;
						SimDateTime.cMinutes = 1;
						
                                                IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, ptrDateTime);
						DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
					}
					++fgptrMoveSequence;						// bump step pointer
					break;


				// *****************************************
				//		Simulator Calculate Sun Position
				// *****************************************
				case MOVE_SEQ_SIM_CALC_SUN_POSITION:				// save current sun position to fgSPAOrientation
					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Calculate Sun Position to SPAOrientation\r\n", WAIT_FOR_DISPLAY);
					#endif

					// bump simulation DateTime
					if (++SimDateTime.cMinutes > DS3232_MAX_MINUTES)
					{
						SimDateTime.cMinutes = DS3232_MIN_MINUTES;

						if (++SimDateTime.cHours > DS3232_MAX_HOURS_24)
						{
							SimDateTime.cHours = DS3232_MIN_HOURS_24;

							if(++SimDateTime.cDate > DS3232_MAX_DAY)	// kludge value, will give 31 days to all months
							{
								SimDateTime.cDate = DS3232_MIN_DAY;

								if (++SimDateTime.cMonth > DS3232_MAX_MONTH)
									SimDateTime.cMonth = DS3232_MIN_MONTH;
							}
						}
					}

					IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, &SimDateTime);
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);


					// calculate and display Sun Position, Global SPA Coordinates
					// CalculateSunPosition() also does the Backtracking calculations, but does NOT make use of them
					/*fgSPAMove.SunPositionState = */ IGNORE_RETURN_VALUE CalculateSunPosition(&fgSPAOrientation, &SimDateTime);
					strcpy(szfnDisplayStr, "SPA\t");
					SPA_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);

                                        // convert SPA orientation to Local Orientation
					ConvertSPAtoLocalOrientation(&fgSPAOrientation);
					// display Local (Panel) orientation
					strcpy(szfnDisplayStr, "SPA Local ");
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);

                                        // Adjust orientation for Backtracking, and display new orientation in SPA coordinates
					IGNORE_RETURN_VALUE AdjustForBacktracking(&fgSPAOrientation);
					strcpy(szfnDisplayStr, "Backtrack ");
					SPA_Backtrack_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);

					// bounds check and adjust Orientation for limits or stow positions (based on LOCAL Panel coordinate system, NOT SPA coordinate system)
					eMoveState = PanelPositionFSM(&fgSPAOrientation);
					strcpy(szfnDisplayStr, "SetPt\t");
					Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &fgSPAOrientation);	// format for display
					DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);

					++fgptrMoveSequence;						// bump step pointer
					break;



				// *****************************************
				//	Set SPA Distance to Move (single axis)
				// *****************************************
				// this step exists to show the distance of the next move.
				// In a single axis system, it may appear redundant
				// in a two axis system, it will display the distance before each move
				case MOVE_SEQ_STEP_SET_SPA_MOVE_DISTANCE:			// set distance to move
					if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
					{
						pglMoveDistanceTicks[AXIS_AZIMUTH] = fgSPAMove.lAzimuthMoveTicks;
						#ifdef USE_INCLINOMETER_FEEDBACK
							pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = ((float)fgSPAMove.lAzimuthMoveTicks / AZ_FLOAT_MSI_TICKS_PER_DEGREE);
						#endif

						#ifdef USE_INCREMENTAL_MOVES
							// adjust move distance for number of incremental moves
							pglMoveDistanceTicks[AXIS_AZIMUTH] /= MOVE_INCREMENT_COUNT;
							pgfMoveDistanceDegrees[MOTOR_AZIMUTH] /= MOVE_INCREMENT_COUNT_FLOAT;
						#endif
                                                #ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
						AddDisplayStr("Set SPA Az Move (ticks):\t");
						INT32StoASCIIstr(pglMoveDistanceTicks[AXIS_AZIMUTH], INT32S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);
						#ifdef USE_INCLINOMETER_FEEDBACK
							AddDisplayStr("\t(Degrees):");
							AddDisplayStr((const char *)ftoa(pgfMoveDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
						#endif
						AddDisplayNewLine();
						DisplayStrWait(SERIAL_MENU_UART);
                                                #endif
					}
					else if (fgptrMoveSequence->eAxis IS AXIS_ELEVATION)
					{
						// ==> needs to handle inclinometer, incremental moves
						pglMoveDistanceTicks[AXIS_ELEVATION] = fgSPAMove.lElevationMoveTicks;
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						AddDisplayStr("Set SPA El Move (ticks):\t");
                                                
						INT32StoASCIIstr(fgSPAMove.lElevationMoveTicks, INT32S_WIDTH, szfnDisplayStr);
						AddDisplayStrAndNewLine(szfnDisplayStr);
						DisplayStrWait(SERIAL_MENU_UART);				// start display (serial output) of line
                                                #endif
                                        }
					else
					{
                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						DisplayMessage(SERIAL_MENU_UART,"Not a Valid Command\r\n", WAIT_FOR_DISPLAY);
                                                #endif
						RuntimeError(MOVE_SEQ_FSM_ERROR_INVALID_STEP);
					}

					++fgptrMoveSequence;						// bump step pointer
					break;

				// *****************************************
				//			Read Inclimometer
				// *****************************************
				#ifdef USE_MMA8452Q_INCLINOMETER
					// called for display purposes ONLY during SPA_TRACKING, value is NOT averaged, so it may not be correct..
					case MOVE_SEQ_STEP_READ_INCLINOMETER:
						{
							INCLINOMETER_SAMPLE Inclination;

							// read and display Inclinometer values. For display ONLY, values are not averaged or used for motion control
							if (ReadInclinometerSample(&Inclination) IS TRUE)
							{
                                                            #ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
								IGNORE_RETURN_VALUE FormatInclination(szfnDisplayStr, &Inclination);
								AddDisplayStrAndNewLine(szfnDisplayStr);
								DisplayStrWait(SERIAL_MENU_UART);				// start display (serial output) of line
                                                            #endif
                                                        }
							else
							{
                                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
								AddDisplayStrAndNewLine("Failed to Read Inclinometer");
                                                              
								DisplayStrWait(SERIAL_MENU_UART);				// start display (serial output) of line
                                                                #endif
                                                        }
						}
						++fgptrMoveSequence;						// bump step pointer
						break;
				#endif		// USE_MMA8452Q_INCLINOMETER


				// *****************************************
				//			Skip on SPA Move Done
				// *****************************************
				// Test for completion of incremental moves within an SPA move
				case MOVE_SEQ_STEP_SPA_SKIP_ON_MOVE_DONE:

					CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					AddDisplayStr("Cur Orientation (degreesSSS): ");
                                        #endif
					CurrentLocalOrientation_Format(szfnDisplayStr);		// get current orientation, formatted for output
					AddDisplayStrAndNewLine(szfnDisplayStr);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                        #endif
					if (eMoveState IS PANEL_POSITION_HOLD)
					{
						#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
							DisplayMessage(SERIAL_MENU_UART,"PANEL_POSITION_HOLD, Skip on Move Complete\r\n", WAIT_FOR_DISPLAY);
						#endif
						++fgptrMoveSequence;					// bump step pointer
						++fgptrMoveSequence;					// bump step pointer
						break;
					}

					// if  moving right
					//		if current angle > stored angle + move angle
					//			skip next step
					// else if  moving left
					//		if current angle < stored angle - move angle
					//			skip next step

					if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
					{

						switch(fgSPAMove.eAzimuthDirection)					// command flag is used ONLY to indicate direction of movement
						{
							case PWM_DIR_FORWARD:
								// if the last move is past the desired movement, move is complete
								if ((fgCurrentLocalOrientation.fAzimuth > (fgOrientation1.fAzimuth + (pgfMoveDistanceDegrees[MOTOR_AZIMUTH] * MOVE_INCREMENT_COUNT)))
									OR		// OR an additional half of an incremental move will put us past the desired movement, move is complete
									((fgCurrentLocalOrientation.fAzimuth + (pgfMoveDistanceDegrees[MOTOR_AZIMUTH] / 2)) > (fgOrientation1.fAzimuth + (pgfMoveDistanceDegrees[MOTOR_AZIMUTH] * MOVE_INCREMENT_COUNT))))
								{
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Skip on Move Complete\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									++fgptrMoveSequence;					// bump step pointer to skip next step
								}
								break;

							case PWM_DIR_REVERSE:
								// NOTE: calculations here are based on pgfMoveDistanceDegrees[MOTOR_AZIMUTH] being a NEGATIVE number, thus the use of fabs() to the math makes sense!
								// if the last move is past the desired movement, move is complete
								if ((fgCurrentLocalOrientation.fAzimuth < (fgOrientation1.fAzimuth - (fabs(pgfMoveDistanceDegrees[MOTOR_AZIMUTH]) * MOVE_INCREMENT_COUNT)))
									OR		// OR an additional half of an incremental move will put us past the desired movement, move is complete
									((fgCurrentLocalOrientation.fAzimuth - (fabs(pgfMoveDistanceDegrees[MOTOR_AZIMUTH]) / 2)) < (fgOrientation1.fAzimuth - (fabs(pgfMoveDistanceDegrees[MOTOR_AZIMUTH]) * MOVE_INCREMENT_COUNT))))
								{
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Skip on Move Complete\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									++fgptrMoveSequence;					// bump step pointer to skip next step
								}
								break;

							default:
                                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
								DisplayMessage(SERIAL_MENU_UART,"Skip on Move Unknown Direction\n", NO_WAIT_FOR_DISPLAY);
                                                                #endif
								break;
						}
					}

					// ==> needs implementation for elevation, even if we do not use it now

					++fgptrMoveSequence;						// bump step pointer
					break;

				// *****************************************
				//			Skip on Move Done
				// *****************************************
				// for testing incremental moves
				case MOVE_SEQ_STEP_SKIP_ON_MOVE_DONE:

					CurrentLocalOrientation_Read(&fgCurrentLocalOrientation);
					AddDisplayStr("Cur Orientation (degreesSS): ");
					CurrentLocalOrientation_Format(szfnDisplayStr);		// get current orientation, formatted for output
					AddDisplayStrAndNewLine(szfnDisplayStr);
                                        #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
					DisplayStrWait(SERIAL_MENU_UART);			// start display (serial output) of line
                                        #endif
					// if  moving right
					//		if current angle > stored angle + move angle
					//			skip next step
					// else if  moving left
					//		if current angle < stored angle - move angle
					//			skip next step

					if (fgptrMoveSequence->eAxis IS AXIS_AZIMUTH)
					{
						switch(fgptrMoveSequence->eCommand)					// command flag is used ONLY to indicate direction of movement
						{
							case EF_MTN_CMD_RUN_FWD:
								// if the last move is past the desired movement, move is complete
								if ((fgCurrentLocalOrientation.fAzimuth > (fgOrientation1.fAzimuth + (pgfMoveDistanceDegrees[MOTOR_AZIMUTH] * MOVE_INCREMENT_COUNT)))
									OR		// OR an additional half of an incremental move will put us past the desired movement, move is complete
									((fgCurrentLocalOrientation.fAzimuth + (pgfMoveDistanceDegrees[MOTOR_AZIMUTH] / 2)) > (fgOrientation1.fAzimuth + (pgfMoveDistanceDegrees[MOTOR_AZIMUTH] * MOVE_INCREMENT_COUNT))))
								{
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Skip on Move Complete\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									++fgptrMoveSequence;					// bump step pointer to skip next step
								}
								break;

							case EF_MTN_CMD_RUN_REV:
								// NOTE: calculations here are based on pgfMoveDistanceDegrees[MOTOR_AZIMUTH] being a NEGATIVE number, thus the use of fabs()
								// if the last move is past the desired movement, move is complete
								if ((fgCurrentLocalOrientation.fAzimuth < (fgOrientation1.fAzimuth - (fabs(pgfMoveDistanceDegrees[MOTOR_AZIMUTH]) * MOVE_INCREMENT_COUNT)))
									OR		// OR an additional half of an incremental move will put us past the desired movement, move is complete
									((fgCurrentLocalOrientation.fAzimuth - (fabs(pgfMoveDistanceDegrees[MOTOR_AZIMUTH]) / 2)) < (fgOrientation1.fAzimuth - (fabs(pgfMoveDistanceDegrees[MOTOR_AZIMUTH]) * MOVE_INCREMENT_COUNT))))
								{
									#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
										DisplayMessage(SERIAL_MENU_UART,"Skip on Move Complete\r\n", NO_WAIT_FOR_DISPLAY);
									#endif
									++fgptrMoveSequence;					// bump step pointer to skip next step
								}
								break;

							default:
                                                            #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
								DisplayMessage(SERIAL_MENU_UART,"Skip on Move Unknown Direction\n", NO_WAIT_FOR_DISPLAY);
                                                                #endif
								break;
						}
					}

					// ==> needs implementation for elevation, even if we do not use it now

					++fgptrMoveSequence;						// bump step pointer
					break;

				// *****************************************
				//		End Placekeeper/Terminator
				// *****************************************
				case MOVE_SEQ_STEP_NONE:
					//BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_STOP);		// force change to STOP

					#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						ClearDisplayStr();
						AddDisplayStr("Done: Total Command Count: ");
						WORDtoASCIIstr(fgwCmdExecCounter, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);
						AddDisplayNewLine();						// add line terminator
						DisplayStr(SERIAL_MENU_UART);				// start display (serial output) of line
					#endif

					fSubStateStatus = SUBSTATE_DONE;			// ready to exit state
					ClearMoveSequenceStarted();					// mark no longer in process. Called here rather than in ST_MOVE_SEQ_STOPPED due to synchronization issues
					break;

				}		// end of switch(fgptrMoveSequence->eStepType)

			break;

		} 	// end of state processing


	// *************************************************
	//		check for any unprocessed events
	// *************************************************
	// we will consider an event unprocessed
	// Note: this means that an asynchronous external event that was generated during THIS pass through the User FSM
	//       will NOT be consider unprocessed during THIS execution pass

	// AND Motion Events upon FSM entry with Motion Events NOW to determine what was NOT processed during this pass
	// note the we are masking out the TOO_FAST and TOO_SLOW events, which may take more than one pass to clear
	efMoveSequenceEventsUnprocessedThisPass = efMoveSequenceEventsUponEntry & efMoveSequenceEvents & EF_MOTION_MULTI_PASS_EVENTS_MASK;	// flags that were set upon entry and are STILL set now will be a 1

	if (efMoveSequenceEventsUnprocessedThisPass IS_NOT 0)									// any unprocessed events?
		{
		RuntimeError(MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT);								// flag runtime error
//		efUnprocessedMoveSequenceEvents |= efMoveSequenceEvents;					// keep a log of unprocessed events
		efUnprocessedMoveSequenceEvents |= efMoveSequenceEventsUnprocessedThisPass;	// keep a log of unprocessed events
		}

#ifdef UNPROCESSED_EVENT_ERROR
	// some events may remain unprocessed for a few passes.. we want to track only those times when there are unprocessed events

	if (efMoveSequenceEvents IS_NOT 0)										// any unprocessed events?
		{
		RuntimeError(MOVE_SEQ_FSM_ERROR_UNPROCESSED_EVENT);					// flag runtime error
		}
#endif
}


// end of MoveSequenceFSM.c

