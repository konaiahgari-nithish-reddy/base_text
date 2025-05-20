// *************************************************************************************************
//										S e r i a l T i m e r . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Serial Communications Timer definitions
//
// *************************************************************************************************

#ifndef SERIALTIMER_H
	#define SERIALTIMER_H
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// I2C timeout value
// Timer 4 counts at 10MHz / 256 = 39.1KHz, 26.6uS period
// I2C bus is nominally 100KHz (could be 400KHz), or 10uS per bit
// considering that most of the timed operations are very low-level, NOT packet-level, a count of 100 should be PLENTY
#define	I2C_TIMEOUT_CNT					100

#define	GetI2CTimeoutState()			(IS_BITSET(efTimerEvents, EF_TIMER_I2C_TIMEOUT))		// macro to query state of timeout

//-------------------------------------------------------------------------------------------------------
// Function Declarations
//-------------------------------------------------------------------------------------------------------

void StartI2CTimeout(void);
void ClearI2CTimeout(void);
void StopI2CTimeout(void);

//-------------------------------------------------------------------------------------------------------
// Global Data
//-------------------------------------------------------------------------------------------------------

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined (DEFINE_GLOBALS)
//
#elif defined (DEFINE_EXTERNS)
//
#endif


// end of SerialTimer.h

