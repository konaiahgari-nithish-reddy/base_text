// *************************************************************************************************
//										D e b u g . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Debug Runtime Error definitions, debug code ring buffer
//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// error bases allow the use of a simple enum to define non-conflicting error sets for each subsystem

#ifndef __DEBUG_H__
#define __DEBUG_H__

//#include"GenericTypeDefs.h"
typedef unsigned char	    BYTE;
//typedef unsigned int	    WORD;
#define	RuntimeError(x)		_RuntimeError((WORD) x);

enum tagErrorBase
{

	ERROR_BASE = 			0,
	SYSTEM_ERROR_BASE = 		0,			// system parameters, flash memory
	DEBUG_ERROR_BASE =		0x0100,
	MAIN_ERROR_BASE	= 		0x0200,
	MENU_FSM_ERROR_BASE =		0x0300,		// serial menu
	MOVE_SEQ_FSM_ERROR_BASE =	0x0400,		// move sequence
	MTN_PHASE_FSM_ERROR_BASE =	0x0500,		// move commands, motion phases
	MTN_LIMIT_ERROR_BASE =		0x0600,		// motion limits
	MTN_FSM_ERROR_BASE =		0x0700,		// motion FSM
	PWM_ERROR_BASE =		0x0800,		// motor PWM
	MTN_SENSOR_ERROR_BASE =		0x0900,		// motion sensors (Hall Effect)
	SER_FSM_ERROR_BASE =		0x0A00,
	SER_DISPLAY_ERROR_BASE =	0x0B00,
	TIMER_ERROR_BASE =	 	0x0C00,
	IS_FSM_ERROR_BASE =		0x0D00,		// input switches
	LED_ERROR_BASE =		0x0E00,
	I2C_ERROR_BASE =		0x0F00,
	SPI_ERROR_BASE =		0x1000,
	UPDATE_PARAM_ERROR_BASE =	0x2000,
	RX_MSG_ERROR_BASE =		0x3000,
	INPUT_SWITCH_ERROR_BASE =	0x4000,
	ADC_ERROR_BASE =		0x5000,
	INCLINOMETER_ERROR_BASE =	0x6000
};

// ADC_ERROR_BASE

// LED display sequence values
#define	LED_FIVE_BLINKS				5
#define	LED_TEN_BLINKS				10
#define	LED_QUARTER_SECOND			11
#define	LED_HALF_SECOND				12
#define	LED_ONE_SECOND				13
#define	LED_QUARTER_SECOND_BLINK	14
#define	LED_HALF_SECOND_BLINK		15
#define	LED_ONE_SECOND_BLINK		16
#define	LED_ON						17
#define	LED_ERROR					18
#define	LED_OFF						0

#define MAX_LED_DISPLAY_VALUE		17

#define	LED_SEQUENCE_MASK			0x7F		// mask to strip off REPEAT_SEQUENCE bit

// masks to OR with LED display sequence values
#define	SINGLE_SEQUENCE				(BYTE)0x00
#define	REPEAT_SEQUENCE				(BYTE)0x80

// I/O Port logic levels to drive DEBUG_LEDx, active LOW (common anode connections!)
#define	DEBUG_LED_OFF				1
#define	DEBUG_LED_ON				0

#define	APP_LED_OFF					1
#define	APP_LED_ON					0

//-------------------------------------------------------------------------------------------------------
// Function Prototypes
//-------------------------------------------------------------------------------------------------------

void InitializeDebug(void);								// Initializes timer module
void DebugLEDsTick(void);								// called on timer ticks to update debug LED display

#ifdef	DEBUG_LEDS
	void DebugSetLED1Value(BYTE nLedSequenceSelection);
	void DebugSetLED2Value(BYTE nLedSequenceSelection);
#else
	#define DebugSetLED1Value(x)
	#define DebugSetLED2Value(x)

#endif 	// DEBUG_LEDS

void InitializeAppLEDs(void);							// initialize Application LED sequences
void AppLEDsTick(void);									// called on timer ticks to update Application LED display
void AppSetLED1Value(BYTE nLedSequenceSelection);
void AppSetLED2Value(BYTE nLedSequenceSelection);


// this macro forces a cast from an arbitrary enum to a BYTE
//#define	RuntimeError(x)		_RuntimeError((unsigned int) x);
#define	RuntimeEvent(x)		_RuntimeError((WORD) x);

// this is actual function prototype
void _RuntimeError(WORD wErrorCode);

void ResetRuntimeError(void);


//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------

#define	ERROR_BUFFER_SIZE		256
#define ERROR_BUFFER_MAX_INDEX	(unsigned int)(ERROR_BUFFER_SIZE - 1)

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

//#if defined (DEFINE_GLOBALS)
	// ring buffer for runtime error codes
//	GLOBAL_INIT	WORD			pgwErrorIndex = 0;
//#elif defined (DEFINE_EXTERNS)
//	GLOBAL		ARRAY	WORD	pgwRuntimeErrorBuffer[];
//	GLOBAL		WORD			pgwErrorIndex;
//#endif

// end of Debug.h

#endif	// #ifndef DEBUG_H
