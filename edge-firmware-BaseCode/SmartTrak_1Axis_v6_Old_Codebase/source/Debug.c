// *************************************************************************************************
//										D e b u g . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Debug Runtime Error Handler functions
//
// *************************************************************************************************

//-----------------------------------------------------------------------------
// #include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

// processor include file
//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include "gsfstd.h"				// gsf standard #defines

#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

//#include "init.h"				// port definitions and initialization state
#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagDebugErrors
{
	DEBUG_ERROR_NONE = DEBUG_ERROR_BASE,// 0
	DEBUG_UNEXPECTED_TICK,				// 1 unexpected timer tick event
	DEBUG_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	DEBUG_ERROR_INVALID_STATE,			// 3 not a valid state
	DEBUG_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	DEBUG_ERROR_INVALID_LED_SEQUENCE,	// 5 not a LED sequence number
	DEBUG_ERROR_STACK_OVERFLOW,			// 6 stack overflow

	DEBUG_UNPROCESSED_EVENT = DEBUG_ERROR_BASE + 0x0F
};


//-------------------------------------------------------------------------------------------------------
// Static Variables
//-------------------------------------------------------------------------------------------------------

#ifdef DEBUG_LEDS
	PRIVATE_INIT BYTE bDebugSequenceCtr1 = 0;			// LED sequence index
	PRIVATE_INIT BYTE bDebugSequenceCtr2 = 0;

	PRIVATE_INIT INT32U ulDebugLED1Sequence = 0L;
	PRIVATE_INIT INT32U ulDebugLED2Sequence = 0L;
#endif	// #ifdef DEBUG_LEDS

#ifdef APP_LEDS
	PRIVATE_INIT BYTE bAppSequenceCtr1 = 0;				// LED sequence index
	PRIVATE_INIT BYTE bAppSequenceCtr2 = 0;

	PRIVATE_INIT INT32U ulAppLED1Sequence = 0L;
	PRIVATE_INIT INT32U ulAppLED2Sequence = 0L;
#endif	// #ifdef APP_LEDS


#define	LED_PATTERN_LEN		(BYTE)20				// LED patterns are 20 bits long because.. they are. 20 bits == 2 seconds

PRIVATE_INIT FILE_GLOBAL ARRAY INT32U ulLEDTable[] =
{

	// this sequence can display from 0 to 10 reasonably fast blinks

	(INT32U)0x00000000,			// 0000 0000 0000 0000 0000		LED_OFF
	(INT32U)0x00000001,			// 0000 0000 0000 0000 0001		1 blink
	(INT32U)0x00000005,			// 0000 0000 0000 0000 0101		2 blinks
	(INT32U)0x00000015,			// 0000 0000 0000 0001 0101
	(INT32U)0x00000055,			// 0000 0000 0000 0101 0101
	(INT32U)0x00000155,			// 0000 0000 0001 0101 0101
	(INT32U)0x00000555,			// 0000 0000 0101 0101 0101
	(INT32U)0x00001555,			// 0000 0001 0101 0101 0101
	(INT32U)0x00005555,			// 0000 0101 0101 0101 0101
	(INT32U)0x00015555,			// 0001 0101 0101 0101 0101
	(INT32U)0x00055555,			// 0101 0101 0101 0101 0101		10 blinks

	(INT32U)0x00000007,			// 0000 0000 0000 0000 0111		LED_QUARTER_SECOND
	(INT32U)0x00000014,			// 0000 0000 0000 0001 1111		LED_HALF_SECOND
	(INT32U)0x000003FF,			// 0000 0000 0011 1111 1111		LED_ONE_SECOND

	(INT32U)0x00033333,			// 0011 0011 0011 0011 0011		LED_QUARTER_SECOND_BLINK
	(INT32U)0x00007C14,			// 0000 0111 1100 0001 1111		LED_HALF_SECOND_BLINK
	(INT32U)0x000003FF,			// 0000 0000 0011 1111 1111		LED_ONE_SECOND_BLINK

	(INT32U)0x000FFFFF,			// 1111 1111 1111 1111 1111		LED_ON

	(INT32U)0x35353535			// catch-all error value


	// perhaps the table could be continued with morse code?

};


#define LED_DISPLAY_ERROR       (INT32U)0x35353535      // this is just a catch-all error value

#define	REPEAT_SEQUENCE_BIT		((INT32U)0x80000000)	// marker bit to select repeating sequence


//-------------------------------------------------------------------------------------------------------
// External Variables
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Static Function Prototypes
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Function Bodies
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
//  InitializeDebug
//
//  Input: 	    none
//  Output:	    none
//  Desc:		Initializes the debug module
//-------------------------------------------------------------------------------------------------------

void InitializeDebug(void)
{
	BYTE	i;

	#ifdef DEBUG_LEDS
		bDebugSequenceCtr1 = 0;			// LED sequence index
		bDebugSequenceCtr2 = 0;

		ulDebugLED1Sequence = ulLEDTable[1];
		ulDebugLED2Sequence = ulLEDTable[2];
	#endif

	// clear the runtime error array
	for (i = 0; i < ERROR_BUFFER_MAX_INDEX; i++)
		{
		pgwRuntimeErrorBuffer[i] = 0;
		}

	pgwErrorIndex = 0;					// initialize index into ring buffer

}


//-------------------------------------------------------------------------------------------------------
//  AppLEDsInit
//
//  Input: 	    none
//  Output:	    none
//  Desc:		Initializes Application LED sequences
//-------------------------------------------------------------------------------------------------------

#ifdef APP_LEDS

	void InitializeAppLEDs(void)
	{

		bAppSequenceCtr1 = 0;			// LED sequence index
		bAppSequenceCtr2 = 0;

		ulAppLED1Sequence = ulLEDTable[5];
		ulAppLED2Sequence = ulLEDTable[10];

	}
#endif	// #ifdef APP_LEDS

//-------------------------------------------------------------------------------------------------------
//  DebugLEDTick
//
//  Input: 		none
//  Output:	
//  Desc:		Update debug LEDs, etc. Called on timer tick, every 100mS
//-------------------------------------------------------------------------------------------------------
#ifdef DEBUG_LEDS

	void DebugLEDsTick(void)
	{

		// step LED sequence
		if (ulDebugLED1Sequence & LBITMASK(bDebugSequenceCtr1))
			DEBUG_LED1 = DEBUG_LED_ON;
		else
			DEBUG_LED1 = DEBUG_LED_OFF;

		if (ulDebugLED2Sequence & LBITMASK(bDebugSequenceCtr2))
			DEBUG_LED2 = DEBUG_LED_ON;
		else
			DEBUG_LED2 = DEBUG_LED_OFF;


		// bump pattern counter, and if the pattern is complete, roll over the counter
		if (++bDebugSequenceCtr1 == LED_PATTERN_LEN)
			{
			bDebugSequenceCtr1 = 0;

			// if the LED pattern IS_NOT specified as REPEAT_SEQUENCE, reset to LED_OFF
			if ((INT32U)(ulDebugLED1Sequence & REPEAT_SEQUENCE_BIT) IS_NOT REPEAT_SEQUENCE_BIT)
				ulDebugLED1Sequence = ulLEDTable[LED_OFF];

			}

		// bump pattern counter, and if the pattern is complete, roll over the counter
		if (++bDebugSequenceCtr2 == LED_PATTERN_LEN)
			{
			bDebugSequenceCtr2 = 0;

			// if the LED pattern IS_NOT specified as REPEAT_SEQUENCE, reset to LED_OFF
			if ((INT32U)(ulDebugLED2Sequence & REPEAT_SEQUENCE_BIT) IS_NOT REPEAT_SEQUENCE_BIT)
				ulDebugLED2Sequence = ulLEDTable[LED_OFF];

			}

	}

#endif 	// DEBUG_LEDS


//-------------------------------------------------------------------------------------------------------
//  AppLEDTick
//
//  Input: 		none
//  Output:	
//  Desc:		Update Application LEDs, etc. Called on timer tick, every 100mS
//-------------------------------------------------------------------------------------------------------

#ifdef APP_LEDS

	void AppLEDsTick(void)
	{

		// step LED sequence
		if (ulAppLED1Sequence & LBITMASK(bAppSequenceCtr1))
			LED1 = APP_LED_ON;
		else
			LED1 = APP_LED_OFF;

		if (ulAppLED2Sequence & LBITMASK(bAppSequenceCtr2))
			LED2 = APP_LED_ON;
		else
			LED2 = APP_LED_OFF;


		// bump pattern counter, and if the pattern is complete, roll over the counter
		if (++bAppSequenceCtr1 == LED_PATTERN_LEN)
			{
			bAppSequenceCtr1 = 0;

			// if the LED pattern IS_NOT specified as REPEAT_SEQUENCE, reset to LED_OFF
			if ((INT32U)(ulAppLED1Sequence & REPEAT_SEQUENCE_BIT) IS_NOT REPEAT_SEQUENCE_BIT)
				ulAppLED1Sequence = ulLEDTable[LED_OFF];

			}

		// bump pattern counter, and if the pattern is complete, roll over the counter
		if (++bAppSequenceCtr2 == LED_PATTERN_LEN)
			{
			bAppSequenceCtr2 = 0;

			// if the LED pattern IS_NOT specified as REPEAT_SEQUENCE, reset to LED_OFF
			if ((INT32U)(ulAppLED2Sequence & REPEAT_SEQUENCE_BIT) IS_NOT REPEAT_SEQUENCE_BIT)
				ulAppLED2Sequence = ulLEDTable[LED_OFF];

			}

	}
#endif	// #ifdef APP_LEDS


//-------------------------------------------------------------------------------------------------------
//  DebugSetLEDxValue
//
//  Input: 	    value to display on LED (index into LED display sequence table)
//  Output:	
//  Desc:		Update debug LED sequences
//
//-------------------------------------------------------------------------------------------------------
#ifdef DEBUG_LEDS

	void DebugSetLED1Value(BYTE nLedSequenceSelection)
	{

		if ((nLedSequenceSelection & LED_SEQUENCE_MASK) <= MAX_LED_DISPLAY_VALUE)
		{
			ulDebugLED1Sequence = ulLEDTable[nLedSequenceSelection & LED_SEQUENCE_MASK];

			// if repeat is requested, mark repeat bit in sequence pattern
			if ((nLedSequenceSelection & REPEAT_SEQUENCE) IS REPEAT_SEQUENCE)
			{
				ulDebugLED1Sequence |= REPEAT_SEQUENCE_BIT;
			}

		}
		else
		{
			ulDebugLED1Sequence = LED_DISPLAY_ERROR;
			RuntimeError(DEBUG_ERROR_INVALID_LED_SEQUENCE);
		}

		// restart sequence
		bDebugSequenceCtr1 = 0;

	}

	void DebugSetLED2Value(BYTE nLedSequenceSelection)
	{

		if ((nLedSequenceSelection & LED_SEQUENCE_MASK) <= MAX_LED_DISPLAY_VALUE)
		{
			ulDebugLED2Sequence = ulLEDTable[nLedSequenceSelection & LED_SEQUENCE_MASK];

			// if repeat is requested, mark repeat bit in sequence pattern
			if ((nLedSequenceSelection & REPEAT_SEQUENCE) IS REPEAT_SEQUENCE)
			{
				ulDebugLED2Sequence |= REPEAT_SEQUENCE_BIT;
			}
		}
		else
		{
			ulDebugLED2Sequence = LED_DISPLAY_ERROR;
			RuntimeError(DEBUG_ERROR_INVALID_LED_SEQUENCE);
		}

		// restart sequence
		bDebugSequenceCtr2 = 0;

	}

#endif 	// DEBUG_LEDS


//-------------------------------------------------------------------------------------------------------
//  AppSetLEDxValue
//
//  Input: 	    value to display on LED (index into LED display sequence table)
//  Output:	
//  Desc:		Update Application LED sequences
//
//-------------------------------------------------------------------------------------------------------

#ifdef APP_LEDS

	// Left, YELLOW LED

	void AppSetLED1Value(BYTE nLedSequenceSelection)
	{

		if ((nLedSequenceSelection & LED_SEQUENCE_MASK) <= MAX_LED_DISPLAY_VALUE)
		{
			ulAppLED1Sequence = ulLEDTable[nLedSequenceSelection & LED_SEQUENCE_MASK];

			// if repeat is requested, mark repeat bit in sequence pattern
			if ((nLedSequenceSelection & REPEAT_SEQUENCE) IS REPEAT_SEQUENCE)
			{
				ulAppLED1Sequence |= REPEAT_SEQUENCE_BIT;
			}

		}
		else
		{
			ulAppLED1Sequence = LED_DISPLAY_ERROR;
			RuntimeError(DEBUG_ERROR_INVALID_LED_SEQUENCE);
		}

		// restart sequence
		bAppSequenceCtr1 = 0;

	}

	// Right, GREEN LED

	void AppSetLED2Value(BYTE nLedSequenceSelection)
	{

		if ((nLedSequenceSelection & LED_SEQUENCE_MASK) <= MAX_LED_DISPLAY_VALUE)
		{
			ulAppLED2Sequence = ulLEDTable[nLedSequenceSelection & LED_SEQUENCE_MASK];

			// if repeat is requested, mark repeat bit in sequence pattern
			if ((nLedSequenceSelection & REPEAT_SEQUENCE) IS REPEAT_SEQUENCE)
			{
				ulAppLED2Sequence |= REPEAT_SEQUENCE_BIT;
			}
		}
		else
		{
			ulAppLED2Sequence = LED_DISPLAY_ERROR;
			RuntimeError(DEBUG_ERROR_INVALID_LED_SEQUENCE);
		}

		// restart sequence
		bAppSequenceCtr2 = 0;

	}
#endif	// #ifdef APP_LEDS



//-------------------------------------------------------------------------------------------------------
//  RuntimeError
//
//  Input: 	    error value
//  Output:	
//  Desc:		writes all runtime error codes to a ring buffer to simplify debugging
//
//-------------------------------------------------------------------------------------------------------

void _RuntimeError(WORD wErrorCode)
{

    // write error code to ring buffer
    pgwRuntimeErrorBuffer[pgwErrorIndex] = wErrorCode;

	// check for end of the buffer
    if (pgwErrorIndex IS ERROR_BUFFER_MAX_INDEX)
		{
		// we just wrote to the end of the buffer, so start over
        pgwErrorIndex = 0;
		}
	else
		{
		// bump ring buffer index for NEXT error code
		++pgwErrorIndex;
		}


	// make sure the NEXT error code is always ZERO
    pgwRuntimeErrorBuffer[pgwErrorIndex] = 0;
}



// reset the RuntimeError buffer index
void ResetRuntimeError()
{

	pgwErrorIndex = 0;						// restart the error buffer
	pgwRuntimeErrorBuffer[0] = 0;

}


// end of debug.c
