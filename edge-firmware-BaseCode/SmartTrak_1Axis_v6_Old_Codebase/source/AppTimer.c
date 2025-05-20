// *************************************************************************************************
//										A p p T i m e r . c
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Application Timer definitions
// *************************************************************************************************

//-----------------------------------------------------------------------------
//								#include files
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// #include files
//-----------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
//	Include Files
//-------------------------------------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include <legacy\int_3xx_4xx_legacy.h>	// required for Input Capture interrupt handlers

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

#include "EventFlags.h"			// event flag definitions and globals
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions, for pgeMotionType

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
#include "TimeDelay.h"			// delay in increments of 10uS

#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

#ifndef USE_MOTION_STALL_COUNTER
	#pragma	message ("Motion Stall Detection DISABLED")
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// see AppTimer.h for enum tagTimerErrors



//-------------------------------------------------------------------------------------------------------
// Static and File-Local Variables
//-------------------------------------------------------------------------------------------------------
FILE_GLOBAL_INIT enum tagMotors eMotionStallMotor = MOTOR_NONE;

//-------------------------------------------------------------------------------------------------------
// Timing Requirements
//-------------------------------------------------------------------------------------------------------

// Counting requirements:
//		count Motion Sensor ticks

// Timing requirements:
//		Core Timer	5mS interrupt (basic 5mS tick) 
//		Core Timer	25mS tick, counted from 5mS interrupt (Core Timer ticks, counted in scheduler loop)
//		Core Timer	100mS tick, counted from 5mS tick in main loop (Core Timer ticks, counted in scheduler loop)
//		TMR1	motion stall timer
//		TMR2	Output Compare/PWM timebase
//		TMR3	Input Capture/Motion Sensor timing timebase
//		TMR3	timebase for measuring Motion Sensor, which can range from xx.xmS (max speed) to predetermined stall time (0.xxxs)
//					this can be done by reading a free-running counter on each Motion Sensor tick count interrupt
//					the math must account for the occasional counter overflow.

//-----------------------------------------------
// For details of Oscillator configuration, see:
//	MCUConfigurationBits.h		config bits set at program time, includes details of documentation

//-----------------------------------------------


// *****************************************************************************
//			M o t i o n S t a l l I n t e r r u p t H a n d l e r( )
// *****************************************************************************

// This interrupt handler is called when Timer1 (the motion stall timer) matches the previously loaded count value
// The count value is loaded at the beginning of each MSI tick cycle

void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

	if (eMotionStallMotor IS MOTOR_NONE)						// check for valid motor selection; file global
	{
		RuntimeError(TIMER_ERROR_INVALID_MTN_STALL_MOTOR);		// not expected motor
		return;
	}

	// ***********************************************
	//		Check for Motor Driver OFF
	// ***********************************************
	// a current spike may cause the motor driver to turn OFF, which will stop the motor
	// if this has occured, the /SF line will be LOW. The driver can be re-enabled by disabling and the enabling the motor driver
     #if defined (DRIV_MC33926)
	if ((ReadBridgeSFPins() & PIN_BRIDGE1_SF) IS 0)		// is /SF LOW?
	{
		DisableMotorDrive();
		Delay10us((UINT32)10);				// delay needed here?
		EnableMotorDrive();
		RuntimeError(TIMER_ERROR_MOTOR_DRIVER_OFF);

		Delay10us((UINT32)10);				// delay needed here?

		// restart the counter
		ClearMotionStallTimer(eMotionStallMotor);

		return;

	}
    #endif

	// ***********************************************
	//		Timer 1 Int (39.1KHz from 10MHz PBCLK)
	// ***********************************************
	// Timer 1 is used as the motion stall timer
	// an interrupt means that a Motion Sensor Tick has timed out - which indicates a Motion Stall
	// Timer 1 is not reloaded here. It is loaded after each Motion Sensor Interrupt tick
	// See MotionSensor.c: MotionSensorInterruptHandler()

	// restart the counter
	ClearMotionStallTimer(eMotionStallMotor);

	// the counter has counted to the match value, so we are CLEARLY stalled
	BITSET(efMotionEvents[eMotionStallMotor], EF_MOTION_STALLED);			// Motion STALLED, send event to Motion FSM

	// this code is ONLY for reporting unexpected Stall situations through the RuntimeError buffer
	// log stall timeout
	switch (pgeMotionType[eMotionStallMotor])
		{
		case MOTION_STARTING:
			RuntimeError(TIMER_ERROR_STARTUP_STALL);
			#ifndef USE_MOTION_STALL_COUNTER		// if motion stall detection is disabled for debugging, remove the event
				BITCLEAR(efMotionEvents[eMotionStallMotor], EF_MOTION_STALLED);
			#endif
			break;

		case MOTION_POWERED:
			RuntimeError(TIMER_ERROR_MOTION_STALL);
			#ifndef USE_MOTION_STALL_COUNTER		// if motion stall detection is disabled for debugging, remove the event
				BITCLEAR(efMotionEvents[eMotionStallMotor], EF_MOTION_STALLED);
			#endif
			break;

		case MOTION_COASTING:			// motor is OFF, some coasting MAY occur
			// a stall condition is expected during Coasting, and is used to determine the end of motion
			RuntimeEvent(TIMER_ERROR_COASTING_STALL);
			break;

		case MOTION_BRAKING:			// motor is OFF, some coasting MAY occur
			// a stall condition is expected during Braking, and is used to determine the end of motion
			RuntimeEvent(TIMER_ERROR_BRAKING_STALL);
			break;

		case MOTION_INIT:				// initial state, only at power up
		case MOTION_STOPPED:			// motion has actually STOPPED (quadrature encoder timeout)
		case MOTION_STALLED:			// system has STALLED; no exit
		default:
			RuntimeError(TIMER_ERROR_UNEXPECTED_STALL);
			break;
		}		// end 	switch(eMotionType)

		// used for Oscilloscope measurements

}


/* *********************************************************************** */
//					Motion Stall Timer (Timer 1)
/* *********************************************************************** */

// Motion Stall Timer: Timer 1, 156KHz

// start the Motion Stall timer - this is called on each MSI tick to restart the timer
// The counter counts UP from this initial value.
// if the counter overflows, it generates an interrupt, which is interpreted as a motion stall

void SetMotionStallCtr(enum tagMotors eMotor, MOTION_PROFILE_SPEED_TYPE NewCount)
{

	if (eMotor IS MOTOR_NONE)									// check for valid motor
	{
		RuntimeError(TIMER_ERROR_INVALID_MTN_STALL_MOTOR);		// not expected motor
		return;
	}

	eMotionStallMotor = eMotor;									// keep track of motor being checked for motion stall; will be used by Timer1Handler

	// ****************************************************
	//			Initialize Timer 1, Interrupts
	// ****************************************************
	// Timer 1 is used for motion stall timing

	// Setup Timer 1
	//	Timer 1 ON
	//	Divide by 256 (note timer input is 10MHZ)
	//  Internal source
	// Counter increments at 26.0uS. The shortest Input Capture cycle we are likely to measure is roughly 600 x 26.0uS, 15.6mS
	// Counter TMR1 is initialized to 0x0000
	// Period (compare) register PR1 is initialized to nNewCount, the value at which an interrupt will be generated

	//		   <<-------------T1CON------------->>     PR1				TMR1 = 0
    OpenTimer1(T1_ON | T1_PS_1_256 | T1_SOURCE_INT, (UINT16)NewCount);

	pgLastMotionStallTimer[eMotor] = NewCount;					// keep a copy of last stall timer value

	BITCLEAR(efMotionEvents[eMotor], EF_MOTION_STALLED);		// Motion cannot be STALLED

	// *********************************
	//	Enable Timer Interrupt
	// *********************************
    // set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2 | T1_INT_SUB_PRIOR_3);

	// start Timer 1 (already done, above)

}


// Called from MotionStallInterruptHandler()
void ClearMotionStallTimer(enum tagMotors eMotor)
{
	if (eMotor IS_NOT eMotionStallMotor)						// check for expected motor
	{
		RuntimeError(TIMER_ERROR_INVALID_MTN_STALL_MOTOR);		// not expected motor
		return;
	}

	TMR1 = 0x0000;						// clear counter
}

void StopMotionStallCtr(enum tagMotors eMotor)
{
	if (eMotor IS_NOT eMotionStallMotor)						// check for expected motor
	{
		RuntimeError(TIMER_ERROR_INVALID_MTN_STALL_MOTOR);		// not expected motor
		return;
	}

	// *********************************
	//	Disable Timer and Interrupts
	// *********************************
	// disable Timer 1 match interrupts
    ConfigIntTimer1(T1_INT_OFF);

	// stop Timer 1
	CloseTimer1();

	eMotionStallMotor = MOTOR_NONE;								// no longer need to track motor
}


// ************************************************************************
//			Motion Sensor Input Capture Interrupt Timing (Timer 3)
// ************************************************************************

// Motion Measurement Timer: Timer 3, 156KHz
// This timer is setup for free running, and is cleared ONLY at the beginning of a move
// Called from MotionFSM() to track motion phase timing

// read the most recent Input Capture value, the time between Motion Sensor ticks, timed by T3 at 156KHz (6.40uS)
//		measure time between ticks
//		track speed
//		detect motion stalls

#ifdef USE_HALL_SENSOR_FEEDBACK
	WORD ReadMSITimerInputCapture(enum tagAxis eAxis)
	{
		// NOTE: we are NOT using the peripheral library function ReadCapture1() because it can return more than one value
		//		AND the documentation is WRONG, it says that ReadCapture1() will 'wait for data', which is WRONG

		WORD wRetVal = 0;

		switch(eAxis)
		{
			case AXIS_AZIMUTH:
				if (AZ_ICCONbits.ICBNE)				// make sure data is available
				{
					wRetVal = AZ_ICBUF;				/* reads the input capture buffer */
				}

				if (AZ_ICCONbits.ICBNE)				// check for more data available
				{
					// clear excess data to prevent repeated RuntimeError() calls
					RuntimeError(TIMER_ERROR_MTN_SENSOR_TICK_OVERRUN);
					while (AZ_ICCONbits.ICBNE)		// clear overrun by reading all available data
						wRetVal = AZ_ICBUF;			/* reads the input capture buffer */
				}

				if (AZ_ICCONbits.ICOV)				// check for FIFO overflow
				{
					RuntimeError(TIMER_ERROR_MTN_SENSOR_FIFO_OVERRUN);

					while (AZ_ICCONbits.ICBNE)		// clear overrun by reading all available data
						wRetVal = AZ_ICBUF;			/* reads the input capture buffer */
				}

				break;

			case AXIS_ELEVATION:
				if (EL_ICCONbits.ICBNE)				// make sure data is available
				{
					wRetVal = EL_ICBUF;				/* reads the input capture buffer */
				}

				if (EL_ICCONbits.ICBNE)				// check for more data available
				{
					// clear excess data to prevent repeated RuntimeError() calls
					RuntimeError(TIMER_ERROR_MTN_SENSOR_TICK_OVERRUN);
					while (EL_ICCONbits.ICBNE)		// clear overrun by reading all available data
						wRetVal = EL_ICBUF;			/* reads the input capture buffer */
				}

				if (EL_ICCONbits.ICOV)				// check for FIFO overflow
				{
					RuntimeError(TIMER_ERROR_MTN_SENSOR_FIFO_OVERRUN);

					while (EL_ICCONbits.ICBNE)		// clear overrun by reading all available data
						wRetVal = EL_ICBUF;			/* reads the input capture buffer */
				}

				break;

			default:
				RuntimeError(TIMER_ERROR_INVALID_AXIS);
				break;
		}

		return wRetVal;

	}

#endif	// USE_HALL_SENSOR_FEEDBACK

	
// ************************************************************************
//					Motion Event Timing (Timer 3)
// ************************************************************************

// Motion Measurement Timer: Timer 3, 39.1KHz
// This timer is setup for free running, and is cleared ONLY at the beginning of a move
// Called from MotionFSM() to track motion phase timing
// Used extensively for pgMotionStats[] timing

WORD ReadMSITimer(enum tagAxis eAxis)
{
	return (ReadTimer3());

}


// clear the Motion Sensor Interrupt (MSI) Input Capture FIFO and timer T3
// Called ONLY at tbe beginning of a move
void ClearMSITimer(enum tagAxis eAxis)
{

	#ifdef USE_HALL_SENSOR_FEEDBACK
		WORD wDummy = 0;

		// *********************************
		//	Clear Input Capture Data
		// *********************************
		switch(eAxis)
		{
			case AXIS_AZIMUTH:

				while (AZ_ICCONbits.ICBNE)		// if any data in the FIFO
					wDummy = AZ_ICBUF;			// clear all data in the FIFO

				break;

			case AXIS_ELEVATION:

				while (EL_ICCONbits.ICBNE)		// if any data in the FIFO
					wDummy = EL_ICBUF;			// clear all data in the FIFO

				break;

			default:
				RuntimeError(TIMER_ERROR_INVALID_AXIS);
				break;
		}
	#endif	// USE_HALL_SENSOR_FEEDBACK

	// *********************************
	//		Clear Timer 3 counter
	// *********************************
	WriteTimer3(0x0000);

}


// end of AppTimer.c
