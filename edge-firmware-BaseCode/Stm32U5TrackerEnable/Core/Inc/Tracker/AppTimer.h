// *************************************************************************************************
//										A p p T i m e r . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Application Timer definitions
//
// *************************************************************************************************

#ifndef APPTIMER_H
	#define APPTIMER_H
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// this definition is here because the error flags may be used by interrupt.c
enum tagTimerErrors
{
	TIMER_ERROR_NONE = TIMER_ERROR_BASE,
	TIMER_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	TIMER_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event, currently used for MC33926 disabled
	TIMER_ERROR_INVALID_STATE,				// 3 not a valid state
	TIMER_ERROR_MOTOR_DRIVER_OFF,			// 4 motor driver OFF detected, probably an electrical noise issue
	TIMER_ERROR_5MS_TICK_OVERRUN,			// 5 unprocessed event flag
	TIMER_ERROR_STARTUP_STALL,				// 6
	TIMER_ERROR_MOTION_STALL,				// 7 Motion Stall interrupt
	TIMER_ERROR_COASTING_STALL,				// 8
	TIMER_ERROR_BRAKING_STALL,				// 9
	TIMER_ERROR_UNEXPECTED_STALL,			// A motion type not valid during interrupt
	TIMER_ERROR_INVALID_AXIS,				// B not a valid axis or motor designation
	TIMER_ERROR_MTN_SENSOR_TICK_OVERRUN,	// C ticks too slow, more data available
	TIMER_ERROR_MTN_SENSOR_FIFO_OVERRUN,	// D FIFO full
	TIMER_ERROR_INVALID_MTN_STALL_MOTOR		// E

};

// interrupt rate
#define INT_PER_SEC			200

// timer values are unsigned ints
// NOTE: these seemingly unneccesary casts are REQUIRED to prevent C18 v3.00 from sign-extending byte values (i.e. 0x00CA becomes 0xFFCA)
#define	TIMER_10_MS						((unsigned int)INT_PER_SEC / (unsigned int)100)				// minimum reliable timing with 5mS clock
#define	TIMER_20_MS						((unsigned int)INT_PER_SEC / (unsigned int)50)
#define	TIMER_30_MS						((unsigned int)INT_PER_SEC / (unsigned int)33)
#define	TIMER_TENTH_SECOND				((unsigned int)INT_PER_SEC / (unsigned int)10)
#define	TIMER_FIFTH_SECOND				((unsigned int)INT_PER_SEC / (unsigned int)5)
#define	TIMER_HALF_SECOND				((unsigned int)INT_PER_SEC / (unsigned int)2)
#define	TIMER_1_SECOND					((unsigned int)INT_PER_SEC)


//*************************************
//		Motion Stall Timing
//*************************************
// the maximum allowable time for an encoder tick is the first tick after starting motion

#define	MAX_MTN_SENSOR_TICK_FOR_STALL		0x5554		// largest usable value, 0xFFFF/3 - 1

//lint -e835 error 835: (Info -- A zero has been given as right argument to operator '+')
// NOTE: this has been kluged a bit, so the Elevation sensor tick value will fit within a UINT16
#define	LONGEST_EXPECTED_MTN_SENSOR_TICK(motor)	((motor == MOTOR_AZIMUTH) ? (MotionProfileSpeedAndPWM[motor][(MIN_SPEED_INDEX * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET]) : (MAX_MTN_SENSOR_TICK_FOR_STALL))

#ifdef DOES_NOT_WORK_CORRECTLY // cannot seem to get this to work..
#if defined (DEFINE_GLOBALS)
	// test longest expected motion sensor tick for possible math error (overflow of UINT16)
	// this can only be evaluated where Globals are defined, so LONGEST_EXPECTED_MTN_SENSOR_TICK will have a value
	#if (((WORD)LONGEST_EXPECTED_MTN_SENSOR_TICK) > (WORD)(0xFFFF / 3))
		#error	Encoder Tick Value out of range, Math Error
	#endif
#endif
#endif

// if we do not get an encoder tick for 3x this period, we are CLEARLY stalled
#define	MTN_SENSOR_STARTUP_STALL_PERIOD(motor)		(LONGEST_EXPECTED_MTN_SENSOR_TICK(motor) * 3)			// 3 times the longest expected encoder period; indicates a motion stall

// during powered motion, the stall period is 1.5x the duration of the previous MSI ticks, which is our speed measurement
//#define	MTN_SENSOR_MOTION_STALL_PERIOD(x)	((x * 3) / 2)				// ==> WARNING: this has the potential for math overflow, resulting in erroneously short stall periods!
#define	MTN_SENSOR_MOTION_STALL_PERIOD(x)	(x * 3)							// ==> WARNING: this has the potential for math overflow, resulting in erroneously short stall periods!

// During coasting and braking, if we do not get an encoder tick for 1.5x this period (105mS), we are CLEARLY stalled
#define	MTN_SENSOR_COASTING_STALL_PERIOD(motor)		((LONGEST_EXPECTED_MTN_SENSOR_TICK(motor) * 3) / 2)	// 1.5 times the longest expected encoder period; indicates a motion stall during coasting

// During soft stall recovery, if we do not get an encoder tick for 3x this period, we are CLEARLY stalled
#define	MTN_SENSOR_RECOVERY_STALL_PERIOD(motor)		(LONGEST_EXPECTED_MTN_SENSOR_TICK(motor) * 3)				// 3 times the startup stall period

//  lint +e835		last of macros using LONGEST_EXPECTED_MTN_SENSOR_TICK

//#define	MTN_SENSOR_MOTION_STOPPED		0xFFFF			// value to indicate no motion in process

//#define	STALL_TMR_MAX_VALUE				0xFFFF			// rollover value for motion sensor tick timer
#define	MTN_SENSOR_TMR_MAX_VALUE		0xFFFF			// rollover value for motion sensor tick timer



//-------------------------------------------------------------------------------------------------------
// Global Data
//-------------------------------------------------------------------------------------------------------

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined (DEFINE_GLOBALS)
	GLOBAL_INIT MOTION_PROFILE_SPEED_TYPE	pgLastMotionStallTimer[NUM_MOTORS] = {0, 0};
#elif defined (DEFINE_EXTERNS)
	GLOBAL MOTION_PROFILE_SPEED_TYPE		pgLastMotionStallTimer[];
#endif

//-------------------------------------------------------------------------------------------------------
// Function Declarations
//-------------------------------------------------------------------------------------------------------
void Timer16Handler(void);
void InitializeTimers(void);
WORD ReadMSITimer(enum tagAxis eAxis);					  // read Timer3 value, Motion Control Event Timing at 6.4uS/Tick
#ifdef USE_HALL_SENSOR_FEEDBACK
	WORD ReadMSITimerInputCapture(enum tagAxis eAxis);		// read input capture value (MSI tick timing at 6.4uS/Tick)
#endif
void ClearMSITimer(enum tagAxis eAxis);

void SetMotionStallCtr(enum tagMotors eMotor, MOTION_PROFILE_SPEED_TYPE NewCount);
void ClearMotionStallTimer(enum tagMotors eMotor);
void StopMotionStallCtr(enum tagMotors eMotor);

// end of AppTimer.h

