// *************************************************************************************************
//										S e r i a l T i m e r . c
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Serial Communications Timers
//
//
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
//#include "MotionProfile.h"		// motion profile data table, movement descriptions
//#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
//#include "MotionFSM.h"			// Motion Control function prototypes and definitions, for pgeMotionType

#include "SerialTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "TimeDelay.h"			// delay in increments of 10uS

#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// see AppTimer.h for enum tagTimerErrors



//-------------------------------------------------------------------------------------------------------
// Static and File-Local Variables
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Timing Requirements
//-------------------------------------------------------------------------------------------------------

// Counting requirements:
//		count Motion Sensor ticks


//-----------------------------------------------
// For details of Oscillator configuration, see:
//	MCUConfigurationBits.h		config bits set at program time, includes details of documentation

//-----------------------------------------------


// *****************************************************************************
//			S e r i a l T i m e r I n t e r r u p t H a n d l e r( )
// *****************************************************************************

// This interrupt handler is called when Timer4 (the serial communicationsl timer) matches the previously loaded count value
// The count value is loaded by calls to StartI2CTimeout()

void __ISR(_TIMER_4_VECTOR, ipl2) Timer4Handler(void)
{
    // clear the interrupt flag
    mT4ClearIntFlag();

	// ***********************************************
	//		Timer 4 Int (39.1KHz from 10MHz PBCLK)
	// ***********************************************
	// Timer 4 is used as the Serial Communications timeout (presently just I2C)
	// an interrupt means that communications event has timed out, waiting for the next event
	// Timer 4 is not reloaded here. It is loaded as needed for timeout detection during communication
	// See I2CBus.c

	// clear and stop the counter
	StopI2CTimeout();

	// the counter has counted to the match value, so we have a TIMEOUT
	BITSET(efTimerEvents, EF_TIMER_I2C_TIMEOUT);
	
}


/* *********************************************************************** */
//					I2C Timeout (Timer 4)
/* *********************************************************************** */

// Motion Stall Timer: Timer 4, 39.1KHz

// start the I2C Timeout Timer
// The counter counts UP from this initial value.
// if the counter matches, it generates an interrupt, which is interpreted as a Timeout

void StartI2CTimeout(void)
{

	// ****************************************************
	//			Initialize Timer4, Interrupts
	// ****************************************************
	// Timer 4 is used for communications timeout timing

	// Setup Timer 4
	//	Timer 4 ON
	//	Divide by 256 (note timer input is 10MHZ)
	//  Internal source
	// Counter increments at 26.0uS. 
	// Counter TMR4 is initialized to 0x0000
	// Period (compare) register PR4 is initialized to I2C_TIMEOUT_CNT, the value at which an interrupt will be generated

	//		   <<-------------T4CON------------->>			PR4				 TMR4 = 0
    OpenTimer4(T4_ON | T4_PS_1_256 | T4_SOURCE_INT, (UINT16)I2C_TIMEOUT_CNT);		// T4_IDLE_CON, T4_GATE_OFF, T4_32BIT_MODE_OFF are defaults (0)

	BITCLEAR(efTimerEvents, EF_TIMER_I2C_TIMEOUT);				// clear any previous timeout
	BITSET(efTimerEvents, EF_TIMER_I2C);						// timer is active

	// *********************************
	//	Enable Timer Interrupt
	// *********************************
    // set up the timer interrupt with a priority of 2
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2 | T4_INT_SUB_PRIOR_3);

	// start Timer 4 (already done, above)

}


void ClearI2CTimeout(void)
{
	TMR4 = 0x0000;											// clear counter
}


void StopI2CTimeout(void)
{
	// *********************************
	//	Disable Timer and Interrupts
	// *********************************
    ConfigIntTimer1(T4_INT_OFF);							// disable Timer 4 match interrupts


	TMR4 = 0x0000;											// clear counter

	// stop Timer 1
	CloseTimer4();

	BITCLEAR(efTimerEvents, EF_TIMER_I2C);					// timer is inactive

}

// end of SerialTimer.c
