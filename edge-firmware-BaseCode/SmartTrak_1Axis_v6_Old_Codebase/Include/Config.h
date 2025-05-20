// *************************************************************************************************
//										C o n f i g . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Compile time configuration Macros

//
// *************************************************************************************************
// ***********************************************
//				Default Location
// ***********************************************
#define Version_SAT "5.6"

// ***********************************************
//				Default Location
// ***********************************************
// location, if none is definded, location will be 0.0, 0.0, off the west coast of Africa
#define LOCATION_HYDERABAD_AP_IN
//#define LOCATION_FREMONT_USA
//#define LOCATION_BEVERLY_MA_USA

// ***********************************************
//				Hardware Platform
// ***********************************************
// hardware platform definition. Define only ONE
//#define	PLATFORM_PIC32_SK
#define	PLATFORM_SMARTTRAK_V1
//#define	PLATFORM_SMARTTRAK_V2			// for SECOND prototype board
#define DRIV_VNH5050A
// enable to display warning messages, Microchip did NOT implement messages correctly
//#define DISPLAY_XC32_MESSAGES

#if defined(PLATFORM_PIC32_SK) && defined(PLATFORM_SMARTTRAK_PROTOTYPE)
	// note syntax is inconsistent with #pragma message
	#error Please define only ONE platform. See config.h
#endif


// ***********************************************
//				Watchdog Timer
// ***********************************************

#define	_WDT
// ***********************************************
//				Zigbee Options
// ***********************************************
//#define UART_DUAL_TEST
//#define RXMSG
#define BOTH_UARTS


#define	DIGI
//#define CC2530
#if defined(DIGI) && defined(CC2530) && defined(UART_DUAL_TEST)
	#error Use Only one type of zigbee communication
#endif
#if !defined(DIGI) && !defined(CC2530)
	#error Use atlease one type of zigbee communication
#endif
// ***********************************************
//				Debug Option
// ***********************************************

//#define MAIN_DEBUG
//#define USE_MOVE_SEQ_FSM_STEP_VERBOSE
//#define USE_MOVE_SEQ_FSM_WAIT_VERBOSE

// ***********************************************
//				Serial Communication
// ***********************************************
#define	USE_REMOTE_COMMANDS					// support serial commands, see RxMessage.c, UpdateParameters.c
//#define USE_SLOW_REMOTE_BAUDRATE
#define	USE_FAST_REMOTE_BAUDRATE

// ***********************************************
//				Tracker Selection
// ***********************************************
//#define USE_SINGLE_VERTICAL_AXIS
#define	USE_SINGLE_POLAR_AXIS
//#define USE_TWO_AXES

#define EXTREME_ANGLES      47.5
// ***********************************************
//				Axis Selection
// ***********************************************
#if defined(USE_SINGLE_VERTICAL_AXIS) || defined(USE_TWO_AXES)
	// the Axis selection affects Move Sequences ONLY (MoveSequenceFSM.c)
	//#define USE_AZIMUTH
	//#define USE_ELEVATION
	#define USE_AZIMUTH_ELEVATION			// two axis

	#if defined(USE_AZIMUTH_ELEVATION)
		#define USE_AZIMUTH
		#define USE_ELEVATION
	#endif
#elif defined(USE_SINGLE_POLAR_AXIS)
	#define USE_AZIMUTH					// single polar axis is effectively azimuth
	#define USE_POLAR_AXIS_MOVE_TABLE	// enable storing a table of moves, see MoveSequenceFSM.h
	#define USE_BACKTRACKING			// enable backtracking
#endif

//#define	RETURN_TO_CENTER			// return to center during find end points sequence


// ***********************************************
//				Drive Selection
// ***********************************************

// drive type for MotionProfile.h table
//#define USE_ELEVATION_SLEW_DRIVE		// use slew drive approximation, 2 ticks per motor rotation
#define USE_ELEVATION_LINEAR_DRIVE		// use linear actuator, 46 ticks per linear inch, 0.158 ticks per motor rotation
//#define USE_ELEVATION_LINEAR_DRIVE_SIMULATOR	// use linear actuator, 46 ticks per linear inch, 0.158 ticks per motor rotation
//#define USE_ELEVATION_ON_OFF_LINEAR_DRIVE	// use linear actuator, ON/OFF relay drive

// enable only ONE of these
//#define	USE_PWM_DUTY_CYCLE				// use PWM for duty cycle control
#define	USE_PWM_ON_OFF					// use PWM 0% and 100% only, for driving VFD and Universal Motor
//#define	USE_RELAY_ON_OFF				// use Relays for ON/OFF control

//#define USE_INCREMENTAL_MOVES				// smaller, incremental moves improve accuracy when using Inclinometer feedback
#define USE_COMPLETE_MOVES

//#define SWITCH_INIT


// ***********************************************
//				Feedback Selection
// ***********************************************

//#define	USE_MOTION_STALL_COUNTER		// enable standard motion stall detection. May be disabled for debugging. See AppTimer.c

// hall effect feedback must be used for 2 axis systems; split systems are not currently supported
// hall effect MAY be used for single axis systems
//#define	USE_HALL_SENSOR_FEEDBACK

// inclinometer feedback is used only for USE_SINGLE_POLAR_AXIS
#define	USE_INCLINOMETER_FEEDBACK

#define	USE_AZIMUTH_HALL_SENSOR_FEEDBACK
#define	USE_AZIMUTH_REED_SWITCH_FEEDBACK
#define	USE_AZIMUTH_COMPASS_FEEDBACK

#define	USE_ELEVATION_HALL_SENSOR_FEEDBACK
#define	USE_ELEVATION_REED_SWITCH_FEEDBACK
#define	USE_ELEVATION_INCLINOMETER_FEEDBACK

// for simulated motion feedback (requires two wires on PIC32 Startup Kit I/O Expansion Board)
//#define	USE_FEEDBACK_SIMULATOR



#if defined(USE_INCLINOMETER_FEEDBACK) && defined(USE_HALL_SENSOR_FEEDBACK)
	#warning Only one feedback type allowed
#endif


// ***********************************************
//
// ***********************************************

#define	_PLIB_DISABLE_LEGACY

// Creates table of motion speed error values during acceleration
#define	MOTION_ERROR_TABLE		// disabled to reduce processing time during Motion Sensor Tick

// For initial testing of new harware; disables most of MotionSensor_Tick() for Open Loop moves ONLY. No Motion Phases..
//#define OPEN_LOOP_MOVES_ONLY

// For initial testing of new harware
//#define UART_TEST

#if defined (PLATFORM_PIC32_SK) && defined (DISPLAY_XC32_MESSAGES)
	#pragma	message ("Build for Microchip PIC32 Starter Kit")
#endif

// these message are left active because they produce special function executables
#ifdef USE_FEEDBACK_SIMULATOR
	#warning Use Motion Feedback Simulator
#endif

#ifdef OPEN_LOOP_MOVES_ONLY
	#warning Open Loop Moves ONLY supported
#endif

// SLEW Gearbox ratios. Define only ONE
//#define SLEW_575_TO_1
#define SLEW_236_TO_1

// ***********************************************
//				I2C definitions
// ***********************************************
// NOTE: Do NOT enable USE_PCA9554_IO if the PCA9554 for switch input is NOT present
//	if the firmware is enabled without the hardware present, access to all other I2C devices will fail

// PCA9554 I/O Expander Enable, Addressing
#if defined(PLATFORM_PIC32_SK)
	#define USE_PCA9554_ADDR			// devices labeled P54, used for PIC32SK I/O Expansion board prototype
	#define USE_PCA9554_IO				// I/O expanders for Switches, LEDs present
#elif defined(PLATFORM_SMARTTRAK_V1)
	#define USE_PCA9554A_ADDR			// devices labeled 54A
	#define USE_DS3232_RTCC				// enable access to RTCC
	#define USE_MMA8452Q_INCLINOMETER	// enable access to inclinometer
#elif defined(PLATFORM_SMARTTRAK_V2)
	#define USE_PCA9554A_ADDR			// devices labeled 54A
	#define USE_DS3232_RTCC				// enable access to RTCC
	#define USE_PCA9554_IO				// I/O expanders for Switches, LEDs present ENABLE **ONLY** IF PRESENT
	#define USE_MMA8452Q_INCLINOMETER	// enable access to inclinometer
#endif

#define USE_I2C_RTCC_BLOCK_WRITE		// single call to write_i2c_device() in WriteRTCCRAMArray() writes all bytes
//#define USE_INCLINOMETER_BLOCK_READ		// single call to write_12c_device() in ReadInclinometerArray() reads all accelerometer
#define USE_I2C_NORMAL_CLOCK			// nomimal 12C clock frequency
//#define USE_I2C_FAST_CLOCK			// fast 12C clock frequency

#if defined(USE_INCLINOMETER_FEEDBACK) && !defined(USE_MMA8452Q_INCLINOMETER)
	#warning USE_INCLINOMETER_FEEDBACK requires USE_MMA8452Q_INCLINOMETER
#endif


// ***********************************************
//			Scope Triggers
// ***********************************************
// Oscilloscope Triggers (Enable only ONE)
//#define USE_5MS_INT_TRIGGER					// for timing 5mS interrupts, normally VERY stable
//#define USE_MOTION_SENSOR_INT_TRIGGER			// for timing MSI interrupts
//#define USE_MOTION_SENSOR_TICK_TRIGGER			// for timing MSI Tick handler
//#define USE_MOTION_SENSOR_TICK_RAM_UPDATE_TRIGGER	// for timing Orientation to RTCC NV RAM update
//#define USE_DEBOUNCE_TRIGGER					// for timing Debounce FSM
//#define USE_INPUT_SWITCH_TRIGGER				// for timing input switch (PCA9554) reads
//#define USE_MOTION_FSM_5MS_TRIGGER
//#define USE_MOTION_PHASE_FSM_TRIGGER
//#define USE_MENU_FSM_TRIGGER
//#define USE_MOTION_FSM_25MS_TRIGGER
//#define USE_MOTION_FSM_TIMER_TRIGGER
//#define USE_MOVE_SEQ_TICK_TRIGGER
//#define USE_MOVE_SEQ_FSM_TRIGGER
#define USE_BUTTON_PROC_FSM_TRIGGER

//#define USE_I2C_START_TRIGGER
//#define USE_I2C_STOP_TRIGGER
// it may be useful to enable BOTH of these
//#define USE_I2C_STOP_AFTER_READ_TRIGGER
//#define USE_I2C_STOP_AFTER_WRITE_TRIGGER

//#define UNPROCESSED_EVENT_ERROR			// check for unprocessed events, may generate a LOT of runtime errors

//#define USE_PGBTERMINATE					// software reset, not fully implemented

//#define USE_HARD_STALL_RESET_TIMER		// timed reset after HARD STALL, not tested or fully implemented

// ***********************************************
//				MoveSequencFSM
// ***********************************************
// control serial output from MoveSequenceFSM()
#define USE_MOVE_SEQ_FSM_VERBOSE			// maximize text output from MoveSequenceFSM() Run, Slew
#define USE_MOVE_SEQ_FSM_SPA_VERBOSE		// maximize text output from MoveSequenceFSM() SPA Tracking
//#define USE_MOVE_SEQ_FSM_TIMER_VERBOSE	// text output during wait for timer messages

//#define USE_DYNAMIC_LEDS					// alternate LED implementation; not fully implemented


// ***********************************************
//				Motion Limits
// ***********************************************
// for MotionLimits.h:
//#define USE_PREVENT_MOVE_PAST_LIMIT		// SetMotionLimits() discards any move past Soft Limits
#define USE_TRUNCATE_MOVE_PAST_LIMIT		// SetMotionLimits() truncates any move past Soft Limits

// end of Config.h
