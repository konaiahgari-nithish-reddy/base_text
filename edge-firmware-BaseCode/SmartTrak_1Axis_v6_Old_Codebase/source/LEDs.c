// *************************************************************************************************
//							B u t t o n P r o c e s s i n g F S M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Button (Input Switch) Processing FSM, User Input

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
#include "Debug.h"
#include "LEDs.h"				// LED display handler function definition, used for stall recovery states
//#include "SmartTrak.h"			// Project wide definitions

#include "PCA9554.h"


#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Static Variables
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------
/*
enum tagLEDErrors
{
	LED_ERROR_NONE = LED_ERROR_BASE,
	LED_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	LED_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	LED_ERROR_INVALID_STATE,			// 3 not a valid state
	LED_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	LED_ERROR_UNKNOWN_COMMAND			// 5 not a valid command
	
};
*/

//-------------------------------------------------------------------------------------------------------
//								I / O  E x p a n d e r  L E D S
//-------------------------------------------------------------------------------------------------------

#ifdef USE_PCA9554_IO			// enable in config.h ONLY if PCA9554 hardware is present

	void IOEXP_LED_ON(enum tagLEDS eLED)
	{
		SetLEDState(GetLEDState() | (BYTE)eLED);

	}

	void IOEXP_LED_OFF(enum tagLEDS eLED)
	{
		SetLEDState(GetLEDState() & ~((BYTE)eLED));

	}
#endif
	
//-------------------------------------------------------------------------------------------------------
//										S e t L E D S
//-------------------------------------------------------------------------------------------------------


// the 'dynamic LED' implementation uses multiple LEDs and blink sequences as debug output
// this is NOT fully implmented, but has been left in place for possible future completion and use

#ifdef USE_DYNAMIC_LEDS

	void SetLEDs(enum tagLEDstates eLEDstate)
		{
			switch(eLEDstate)
				{
				case LEDS_OFF:
					break;

				case LEDS_INIT:
					LeftLED(5 | SINGLE_SEQUENCE);
					RightLED(LED_HALF_SECOND | SINGLE_SEQUENCE);

					TickLED(LED_QUARTER_SECOND_BLINK | REPEAT_SEQUENCE);
					StallLED(10 | SINGLE_SEQUENCE);
					break;

				case LEDS_FORWARD:				// left
					LeftLED(LED_ON | REPEAT_SEQUENCE);
					RightLED(LED_OFF | SINGLE_SEQUENCE);

					StallLED(LED_OFF | REPEAT_SEQUENCE);			// turn STALLED/NO MOVE LED off
					break;

				case LEDS_REVERSE:				// right
					RightLED(LED_ON | REPEAT_SEQUENCE);
					LeftLED(LED_OFF | SINGLE_SEQUENCE);

					StallLED(LED_OFF | REPEAT_SEQUENCE);			// turn STALLED/NO MOVE LED off
					break;

				case LEDS_NO_MOVE:
					StallLED(LED_FIVE_BLINKS | REPEAT_SEQUENCE);	// NO MOVE indication

					LeftLED(LED_OFF | SINGLE_SEQUENCE);			// turn off direction LEDs
					RightLED(LED_OFF | SINGLE_SEQUENCE);
					break;

				case LEDS_END_OF_TRAVEL:
					StallLED(LED_FIVE_BLINKS | SINGLE_SEQUENCE);
					break;

				case LEDS_STOPPED:
					LeftLED(LED_OFF | SINGLE_SEQUENCE);			// turn off direction LEDs
					RightLED(LED_OFF | SINGLE_SEQUENCE);

					// STOPPED does NOT override the state of STALLED/NO MOVE - only motion does!
					break;

				case LEDS_SOFT_STALL:
					StallLED(LED_TEN_BLINKS | REPEAT_SEQUENCE);

					LeftLED(LED_OFF | SINGLE_SEQUENCE);			// turn off direction LEDs
					RightLED(LED_OFF | SINGLE_SEQUENCE);
					break;


				case LEDS_HARD_STALL:
					StallLED(LED_QUARTER_SECOND_BLINK | REPEAT_SEQUENCE);

					LeftLED(LED_OFF | SINGLE_SEQUENCE);			// turn off direction LEDs
					RightLED(LED_OFF | SINGLE_SEQUENCE);
					break;

				case LEDS_STACK_ERROR:
					StallLED(LED_ERROR | REPEAT_SEQUENCE);

					LeftLED(LED_ERROR | REPEAT_SEQUENCE);
					RightLED(LED_ERROR | REPEAT_SEQUENCE);
					break;


				default:
					RuntimeError(LED_ERROR_UNKNOWN_COMMAND);
					break;
				}


	}

#endif		//  USE_DYNAMIC_LEDS

// end of LEDs.c
