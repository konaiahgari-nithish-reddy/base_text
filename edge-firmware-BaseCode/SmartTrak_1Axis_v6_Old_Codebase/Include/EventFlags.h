// *************************************************************************************************
//										E v e n t F l a g s . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Event Flag definitions

//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// NOTE: the enum values defined here are used INDIRECTLY. They are the BIT POSITIONS within 16 bit words, 
// which limits each set to 16 values


// NOTE: not all enums are used
//lint -e758			error 758: (Info -- global enum 'foo' (line 27, file moo.h) not referenced)


// ADC Events (minimal because sequencing is done in hardware)
// Event Flag Register : efAdcEvents
enum tagADCEvents
{
    EF_ADC_NULL_EVENT,
    EF_ADC_CONVERSION_DONE
};

// Motion Sensor Events
// One Event Flag Register per motor (Axis) : efMotionSensorEvents[NUM_MOTORS]
enum tagMotionSensorEvents
{
    EF_MOTION_SENSOR_NULL_EVENT,
	EF_MOTION_SENSOR_TICK,
	EF_MOTION_SENSOR_ACTIVE

};

// Motion Limit Sensor Events
// These events cover the sensors separately, so there is only one Event Flag Register : efLimitSensorEvents
// any EOT detection will cause an ALL STOP on both axis
/*
enum tagLimitSensorEvents
{
    EF_LIMIT_SENSOR_NULL_EVENT,
	EF_LIMIT_SENSOR_EL_EOT_UP,
	EF_LIMIT_SENSOR_EL_EOT_DOWN,
	EF_LIMIT_SENSOR_AZ_EOT_RIGHT,
	EF_LIMIT_SENSOR_AZ_EOT_LEFT
};
*/

// PWM Events
// These events cover the sensors separately, so there is only one Event Flag Register : efPWMEvents
/*
enum tagPWMEvents
{
    EF_PWM_NULL_EVENT,
	EF_PWM_EL_TICK,
	EF_PWM_AZ_TICK,
	EF_PWM_EL_FAULT,
	EF_PWM_AZ_FAULT,
};
*/

// Move Sequence Commands
// Move Sequences may be selected from the Serial Menu, a Serial Command, or a button press
// A Move Sequence is one or more  complete movements, and may be one or two axis
// these are system level commands, so there is only one Event Flag Register : efPMoveSequenceEvents
enum tagMoveSequenceEvents
{
	EF_MOVE_SEQ_NONE,				// 0001 NULL command, placekeeper
	EF_MOVE_SEQ_FIND_END_POINTS,	// 0002
	EF_MOVE_SEQ_MOVE_TO_POSITION,	// 0004 two axis move
	EF_MOVE_SEQ_MOVE_TO_NIGHT_STOW,	// 0008	directed EF_MOVE_SEQ_MOVE_TO_POSITION
	EF_MOVE_SEQ_MOVE_TO_WIND_STOW,	// 0010 directed EF_MOVE_SEQ_MOVE_TO_POSITION
	EF_MOVE_SEQ_SPA_SIM,			// 0020 SPA movement simulation, no calculations
	EF_MOVE_SEQ_SPA_CALCULATE,		// 0040 SPA calculations only, no movement
	EF_MOVE_SEQ_SPA_TRACK,			// 0080 SPA Tracking, calculated movements
	EF_MOVE_SEQ_RUN_UP,				// 0100
	EF_MOVE_SEQ_RUN_DOWN,			// 0200
	EF_MOVE_SEQ_RUN_RIGHT,			// 0400
	EF_MOVE_SEQ_RUN_LEFT,			// 0800
	EF_MOVE_SEQ_INC_RUN_RIGHT,		// 1000
	EF_MOVE_SEQ_INC_RUN_LEFT,		// 2000
	EF_MOVE_SEQ_STOP,				// 4000	stop BOTH axes
	EF_MOVE_SEQ_RESET				// 8000 reset and restart
};

// Serial Menu OR MoveSequenceFSM Command Events - start (or stop) movements
// Move Commands are singular moves - one direction, either a fixed distance or until the End-of-Travel sensor is hit
// A Move is a complete movement consisting of all (or almost all) motion phases: accel, run some distance, decel, brake, stop
// One Event Flag Register per motor : efMotionPhaseCommands[NUM_MOTORS]
// NOTE: Serial menu includes additional 'commands' that are just fixed distance moves
enum tagMotionPhaseCommands
{
	EF_MTN_CMD_NONE,				// 0001 NULL command, placekeeper
	EF_MTN_CMD_RUN,					// 0002	full speed, otherwise unspecified, used in MoveSequenceFSM ONLY
	EF_MTN_CMD_RUN_FWD,				// 0004	full speed, length of move must be specified
	EF_MTN_CMD_RUN_REV,				// 0008 full speed, length of move must be specified
	EF_MTN_CMD_SLEW_FWD_TO_END,		// 0010	half speed, no deceleration, used to find limit switch end points ONLY
	EF_MTN_CMD_SLEW_REV_TO_END,		// 0020 half speed, no deceleration, used to find limit switch end points ONLY
	EF_MTN_CMD_STOP,				// 0040
	EF_MTN_CMD_RESET				// 0080	reset and restart
};


// Motion Events
// the same event flag values are used for both motors
// One Event Flag Register per motor : efMotionEvents[NUM_MOTORS]
enum tagMotionEvents
{									//		Event Source
    EF_MOTION_NULL_EVENT,			// 0001
	EF_MOTION_TOO_FAST,				// 0002	Motion Sensor Tick
	EF_MOTION_TOO_SLOW,				// 0004	Motion Sensor Tick
	EF_MOTION_MINIMUM_PWM,			// 0008 PWM_SetDutyCycle()
	EF_MOTION_MAXIMUM_PWM,			// 0010 PWM_SetDutyCycle()
	EF_MOTION_STALLED,				// 0020 ???
	EF_MOTION_END_OF_TRAVEL,		// 0040 Motion Sensor Tick (EOT Sensor)
    EF_MOTION_ACCELERATE_CMD,		// 0080 MotionPhaseFSM
    EF_MOTION_DECELERATE_CMD,		// 0100 MotionPhaseFSM
	EF_MOTION_MSI_COUNT_DONE,		// 0200 Motion Sensor Int
	EF_MOTION_PWM_UPDATE			// 0400 Motion Sensor Tick
	#ifdef USE_INCLINOMETER_FEEDBACK
		,EF_MOTION_ANGLE_MOVE_DONE	// 0800 At destination angle
	#endif

};


// Motion Result Events, passed from Motion FSM to Command FSM
// the same event flag values are used for both motors
// One Event Flag Register per motor : efMotionResultEvents[NUM_MOTORS]
enum tagMotionResultEvents
{
	EF_RESULT_NULL_EVENT,			// 0001
	EF_RESULT_ACCELERATION_DONE,	// 0002
	EF_RESULT_CONSTANT_SPEED_DONE,	// 0004
	EF_RESULT_FWD_LIMIT,			// 0008
	EF_RESULT_REV_LIMIT,			// 0010
	EF_RESULT_STOPPED,				// 0020
	EF_RESULT_SOFT_STALL,			// 0040
	EF_RESULT_HARD_STALL			// 0080
};

// some events may take more than one pass through the FSM to clear, and we do not want to treat them as sticky
#define	EF_MOTION_MULTI_PASS_EVENTS_MASK		(~(BITMASK(EF_MOTION_TOO_SLOW) | BITMASK(EF_MOTION_TOO_FAST)))


// Motion Timer Events
// One Event Flag Register per motor : efMotionTimerEvents[NUM_MOTORS], one per axis
enum tagMotionTimerEvents
{
	EF_MTN_TIMER,					// 0001 Motion timer is active
	EF_MTN_TIMER_25MS_TICK			// 0002 Motion has timed out
//	EF_MTN_TIMER_HALL_MOTOR_SENSOR,	// 0008 Hall Motor Sensor timer is active	do we need separate flags for each motor?
//	EF_MTN_TIMER_HALL_MOTOR_SENSOR_TIMEOUT,	// 0010 Hall Motor Sensor has timed out
//	EF_MTN_TIMER_HARD_STALL			// 2000
};



// button (switch) Events
// application specific definitions are in Include\Debounce.h
// Event Flag Register : efButtonEvents
enum tagSwitchEvents
{
    EF_SWITCH_1_CLOSED_EVENT,
    EF_SWITCH_2_CLOSED_EVENT,
    EF_SWITCH_3_CLOSED_EVENT,
    EF_SWITCH_4_CLOSED_EVENT,
    EF_SWITCH_5_CLOSED_EVENT,
    EF_SWITCH_6_CLOSED_EVENT,
    EF_SWITCH_7_CLOSED_EVENT,
    EF_SWITCH_8_CLOSED_EVENT,
    EF_SWITCH_9_CLOSED_EVENT,
    EF_SWITCH_1_OPEN_EVENT,
    EF_SWITCH_2_OPEN_EVENT,
    EF_SWITCH_3_OPEN_EVENT,
    EF_SWITCH_4_OPEN_EVENT,
    EF_SWITCH_5_OPEN_EVENT,
    EF_SWITCH_6_OPEN_EVENT,
    EF_SWITCH_7_OPEN_EVENT,
    EF_SWITCH_8_OPEN_EVENT,
    EF_SWITCH_9_OPEN_EVENT
};

#define EF_SWITCH_CLOSED_EVENTS_MASK	0x00FF
#define EF_SWITCH_OPEN_EVENTS_MASK		0xFF00

enum tagVirtualSwitchEvents
{
    EF_VIRTUAL_SWITCH_9_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_10_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_11_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_12_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_13_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_14_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_15_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_16_CLOSED_EVENT,
    EF_VIRTUAL_SWITCH_9_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_10_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_11_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_12_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_13_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_14_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_15_OPEN_EVENT,
    EF_VIRTUAL_SWITCH_16_OPEN_EVENT
};



// Timer Events
// Event Flag Register : efTimerEvents
enum tagTimerEvents
{
	EF_TIMER_5MS_TICK_INT,				// 0001 timer interrupt has occured
	EF_TIMER_10MS_TICK_INT,				// 0002 timer interrupt has occured
	EF_TIMER_SECOND_TICK,				// 0004
	EF_TIMER_COMMAND,					// 0080 command timer is active				(not presently used)
	EF_TIMER_COMMAND_TIMEOUT,			// 0100 command timeout complete			(not presently used)
	EF_TIMER_RS232,						// 0200 RS-232 timer is active				(not presently used)
	EF_TIMER_RS232_TIMEOUT,				// 0400 RS-232 TX or RX has timed out		(not presently used)
	EF_TIMER_I2C,						// 0800 I2C timer is active                 (not presently used)
	EF_TIMER_I2C_TIMEOUT,				// 1000 I2C has timed out                   (not presently used)
	EF_TIMER_HARD_STALL					// 2000

};


// the scheduler events are off the timer events, to distrubute processing across time
enum tagSchedulerEvents
{
	EF_SCHED_25MS_TICK1,		// 0001
	EF_SCHED_25MS_TICK2,		// 0002
	EF_SCHED_25MS_TICK3,		// 0004
	EF_SCHED_25MS_TICK4,		// 0008
	EF_SCHED_25MS_TICK5,		// 0010

	EF_SCHED_100MS_TICK1,		// 0020
	EF_SCHED_100MS_TICK2,		// 0040
	EF_SCHED_100MS_TICK3,		// 0080
	EF_SCHED_100MS_TICK4,		// 0100
	EF_SCHED_100MS_TICK5		// 0200
};


enum tagSunPositionEvents
{
	EF_PANEL_POS_WIND_STOW,		// 0001
	EF_PANEL_POS_END_WIND_STOW	// 0002
};


//lint +e758


//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined (DEFINE_GLOBALS)
//	GLOBAL_INIT	EVENTFLAGS  efRS232Events = 0;							// UART
	GLOBAL_INIT	EVENTFLAGS  efADCEvents = 0;							// ADC
	GLOBAL_INIT	EVENTFLAGS  efMotionSensorEvents[NUM_MOTORS] = {0, 0};	// Elevation and Azimuth Motion Sensors, axis specific
	GLOBAL_INIT	EVENTFLAGS  efLimitSensorEvents = 0;					// Limit Sensors

	GLOBAL_INIT EVENTFLAGS  efMoveSequenceEvents = 0;					// system level move sequence request flags
	GLOBAL_INIT	EVENTFLAGS  efMotionPhaseCommands[NUM_MOTORS] = {0, 0};	// system level Move Commands, axis specific (debug/development control and display)
	GLOBAL_INIT	EVENTFLAGS  efMotionEvents[NUM_MOTORS] = {0, 0};		// Motion events, multiple sources, used in MotionFSM
	GLOBAL_INIT	EVENTFLAGS  efMotionResultEvents[NUM_MOTORS] = {0, 0};	// MotionFSM result passed to MotionPhaseFSM, motor specific
	GLOBAL_INIT	EVENTFLAGS  efMotionTimerEvents[NUM_MOTORS] = {0, 0};	// motion timers

	GLOBAL_INIT	EVENTFLAGS  efSwitchEvents = 0;							// switches
	GLOBAL_INIT	EVENTFLAGS  efVirtualSwitchEvents = 0;					// virtual switches: STOP, SPA Tracking
	GLOBAL_INIT	EVENTFLAGS  efTimerEvents = 0;							// system timers
	GLOBAL_INIT	EVENTFLAGS  efSchedulerEvents = 0;						// scheduler
	GLOBAL_INIT	EVENTFLAGS  efPanelPositionEvents = 0;					// Sun Position FSM
#elif defined (DEFINE_EXTERNS)
//	GLOBAL	EVENTFLAGS  efRS232Events;
	GLOBAL	EVENTFLAGS  efADCEvents;
	GLOBAL	EVENTFLAGS  efMotionSensorEvents[];
	GLOBAL	EVENTFLAGS  efLimitSensorEvents;

	GLOBAL	EVENTFLAGS  efMoveSequenceEvents;
	GLOBAL	EVENTFLAGS  efMotionPhaseCommands[];
	GLOBAL	EVENTFLAGS  efMotionEvents[];
	GLOBAL	EVENTFLAGS  efMotionResultEvents[];
	GLOBAL	EVENTFLAGS  efMotionTimerEvents[];

	GLOBAL	EVENTFLAGS  efSwitchEvents;
	GLOBAL	EVENTFLAGS  efVirtualSwitchEvents;
	GLOBAL	EVENTFLAGS  efTimerEvents;
	GLOBAL	EVENTFLAGS  efSchedulerEvents;
	GLOBAL	EVENTFLAGS  efPanelPositionEvents;
#endif


// end of EventFlags.h
