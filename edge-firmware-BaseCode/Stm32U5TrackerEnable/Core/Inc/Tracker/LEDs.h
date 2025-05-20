// *************************************************************************************************
//										L E D s . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Application LED display handler
//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagLEDS
{
	LED1 = 0x01,
	LED2 = 0x02,
	LED3 = 0x04,
	LED4 = 0x08,
	LED5 = 0x10,
	LED6 = 0x20,
	LED7 = 0x40,
	LED8 = 0x80
};

// prototypes for functions to control I/O Expander LEDs
void IOEXP_LED_ON(enum tagLEDS eLED);
void IOEXP_LED_OFF(enum tagLEDS eLED);

// define away I/O Expander LED function calls if hardware not present
// this is IMPORTANT; the initial I2C bus implementation does NOT handle talking to non-existent hardware correctly
#ifndef USE_PCA9554_IO
	#define	IOEXP_LED_ON(x)
	#define	IOEXP_LED_OFF(x)
#endif


// the 'dynamic LED' implementation uses multiple LEDs and blink sequences as debug output
// this is NOT fully implmented, but has been left in place for possible future completion and use
#ifdef USE_DYNAMIC_LEDS
	enum tagLEDstates
	{
		LEDS_OFF = 0,
		LEDS_INIT,
		LEDS_FORWARD,
		LEDS_REVERSE,
		LEDS_NO_MOVE,
		LEDS_END_OF_TRAVEL,
		LEDS_STOPPED,
		LEDS_SOFT_STALL,
		LEDS_HARD_STALL,
		LEDS_STACK_ERROR
	};

	void SetLEDs(enum tagLEDstates eLEDstate);



	// LED designations for PICDEM LCD2 Board (as modified)

	//	Left (Yellow)	Right (Green)	Tick (Red)	Stall (Red)
	//	LED1			LED2			DEBUG_LED1	DEBUG_LED2		see init.h

	#define LeftLED(x)		AppSetLED1Value(x)
	#define RightLED(x)		AppSetLED2Value(x)

	#define TickLED(x)		DebugSetLED1Value(x)
	#define StallLED(x)		DebugSetLED2Value(x)

#endif


// end of LEDs.h
