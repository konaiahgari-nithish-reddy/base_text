// *************************************************************************************************
//									H a r d w a r e P r o f i l e . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	PIC32MX360F512L MCU Definitions and I/O Port Assignments
//
// *************************************************************************************************

// this file originated as Microchip supplied HardwareProfile_PIC32_STK.h, but little of the original file remains.

#ifndef __HARDWARE_PROFILE_H
    #define __HARDWARE_PROFILE_H

//#ifndef __PIC32MX__
//    #error  "Firmware source code supports PIC32 ONLY"
//#endif

// ******************************************************************************
//							MCU CLocks
// ******************************************************************************

/**********************************************************
* GetSystemClock() returns system clock frequency.
*
* GetPeripheralClock() returns peripheral clock frequency.
*
* GetInstructionClock() returns instruction clock frequency.
*
**********************************************************/

/*********************************************************
* Macro: #define	GetSystemClock() 
*
* Overview: This macro returns the system clock frequency in Hertz.
*			* value is 8 MHz x 4 PLL for PIC24
*			* value is 8 MHz/2 x 18 PLL for PIC32
*
*********************************************************/

// System Clock: 80 MHz ((8MHz Crystal/ FPLLIDIV) * FPLLMUL) / FPLLODIV)
// see MCUConfigurationBits.h

#define GetSystemClock()    (80000000ul)
#define SYS_FREQ 			(80000000)

/*********************************************************************
* Macro: #define	GetPeripheralClock() 
*
* Overview: This macro returns the peripheral clock frequency 
*			used in Hertz.
*			* value for PIC24 is <PRE>(GetSystemClock()/2) </PRE> 
*			* value for PIC32 is <PRE>(GetSystemClock()/(1<<OSCCONbits.PBDIV)) </PRE>
*
********************************************************************/
#define GetPeripheralClock()    (GetSystemClock() / (1 << OSCCONbits.PBDIV))

/*********************************************************************
* Macro: #define	GetInstructionClock() 
*
* Overview: This macro returns instruction clock frequency 
*			used in Hertz.
*			* value for PIC24 is <PRE>(GetSystemClock()/2) </PRE> 
*			* value for PIC32 is <PRE>(GetSystemClock()) </PRE> 
*
********************************************************************/
#define GetInstructionClock()   (GetSystemClock())


/* ################################################################## */

//	SmartTrak PLATFORM_SMARTTRAK_V1 Hardware MCU Connections
//	Designator Type Usage Package
//	U1		MC33926		Azimuth H-Bridge				RB2 (/SF), OC2 (IN2), OC1(IN1), RE1 (/D2), RE0(D1), RF0 (EN), RG3 (SLEW), RG2 (INV), RB4 (FB)
//	U2		MC33926		Elevation H-Bridge				RB3 (/SF), OC4 (IN2), OC3(IN1), RE3 (/D2), RE2 (D1), RF1 (EN), RE5 (SLEW), RE6 (INV), RB5 (FB)
//	U3		MAX5035		DC-DC 24V to 5V Step Down		RB9
//	U4		LT3009		DC-DC 5V to 3.3V LDO			RG6 (/SHDN)
//	U5		MAX3221		RS-232 Level Shift				RF2 (Rx Out), RF3 (Tx In)
//	U6		MAX3535		RS-485 Interface/Level Shift	RF4, RF5, RA9, RA15
//	U7		RS-485		Isolation Transformer
//	U8		UCA9554		I/O Expander for Switches(I2C)	RA2 (SCL), RA3 (SDA), RE8 (INT)
//	U9		UCA9554		I/O Expander for LEDs (12C)		RA2 (SCL), RA3 (SDA)
//	U10		DS3232		RTCC (I2C)						RA2 (SCL), RA3 (SDA), RC2 (Reset)
//	U11		SST25VF016B Flash Memory (SPI)				RA14 (/CE), RF6 (SCK), RF7(SO), RF8 (SI), RG0 (/HOLD)

//	Other Connections to MCU
//	AZ0		Azimuth Hall Effect A						RD10
//	AZ1		Azimuth Hall Effect B						RD11
//	EL0		Elevation Hall Effect A						RD8
//	EL0		Elevation Hall Effect A						RD9
//	J9		Screw Terminals	(extra connections)			RB10, RB11, RB12, RB13, RB14, RB15

/*********************************************************************
* IOS FOR THE FLASH/EEPROM SPI (MCU SPI1)
*********************************************************************/
//// see SST25VF016.h for standard SPI bus definitions
//#define USE_SST25_SPI1

//#if defined (USE_SST25_SPI1)  && defined (DISPLAY_XC32_MESSAGES)
//	#warning SPI Flash on SPI
//#endif

////	U11		SST25VF016B Flash Memory (SPI)				RA14 (/CE), RF6 (SCK), RF7(SO), RF8 (SI), RG0 (/HOLD)

//#if defined (PLATFORM_PIC32_SK)
//	// definitions here are ONLY for pins that are NOT part of the standard SPI bus
//	#define SST25_CS_TRIS   TRISFbits.TRISF2                // for PICTail Demo board
//	#define SST25_CS_LAT    LATFbits.LATF2

//	#define SST25_HOLD_TRIS TRISFbits.TRISF1                // for PICTail Demo board
//	#define SST25_HOLD_LAT  LATFbits.LATF1

//#elif defined (PLATFORM_SMARTTRAK_V1)
//	#define SST25_CS_TRIS   TRISAbits.TRISA14				// for SmartTrak Hardware
//	#define SST25_CS_LAT    LATAbits.LATA14

//	#define SST25_HOLD_TRIS TRISGbits.TRISG0				// for SmartTrak Hardware
//	#define SST25_HOLD_LAT  LATGbits.LATG0
//#else
//	#error Platform must be defined
//#endif		// PLATFORM_SMARTTRAK_V1

///*********************************************************************
//*	IOS FOR THE I2C BUS (MCU I2C2)
//*********************************************************************/

//#define	DS3232_I2C_BUS					I2C2
//#define	LED_IO_EXPANDER_I2C_BUS			I2C2
//#define	SWITCH_IO_EXPANDER_I2C_BUS		I2C2
//#define MMA8452Q_I2C_BUS				I2C2

//// PCA9554 I/O Expander Addressing
//// NOTE: addresses here are UNSHIFTED 7 bit addresses
//// device address as set by A2 A1 A0 pins will be added to this
//#if defined (USE_PCA9554_ADDR)						// devices labeled P54
//	#define	PCA_9554_IC_ADDR			0x20		// 7 bit address is 0100 A2 A1 A0
//#elif defined (USE_PCA9554A_ADDR)					// devices labeled 54A
//	#define	PCA_9554_IC_ADDR			0x38		// 7 bit address is 0111 A2 A1 A0
//#else
//	#error PCA9554 I/O Expander Addressing must be defined
//#endif

//// device addresses, as set by logic levels at pins A2, A1, A0
//#if defined (PLATFORM_PIC32_SK)
//	#define	LED_IO_EXPANDER_PCB_ADDR		0x07		// set by PCB traces
//	#define	SWITCH_IO_EXPANDER_PCB_ADDR		0x07		// set by PCB traces, single device shared on PIC32_SK
//#elif defined (PLATFORM_SMARTTRAK_V1)
//	#define	LED_IO_EXPANDER_PCB_ADDR		0x07		// set by PCB traces
//	#define	SWITCH_IO_EXPANDER_PCB_ADDR		0x06		// set by PCB traces
//#else
//	#error Platform must be defined
//#endif		// PLATFORM_SMARTTRAK_V1

//// To create the address written the device, these bits are shifted one position LEFT, and the LSB is the R/NOT_W bit
//#define	LED_IO_EXPANDER_I2C_ADDR		(PCA_9554_IC_ADDR + LED_IO_EXPANDER_PCB_ADDR)
//#define	SWITCH_IO_EXPANDER_I2C_ADDR		(PCA_9554_IC_ADDR + SWITCH_IO_EXPANDER_PCB_ADDR)

//#if defined(USE_I2C_NORMAL_CLOCK)
//	#define	I2C_CLOCK_FREQ					100000		// nomimal 12C clock frequency
//#elif defined (USE_I2C_FAST_CLOCK)
//	#define	I2C_CLOCK_FREQ					400000		// fast 12C clock frequency
//#else
//	#error I2C CLock must be defined
//#endif

//// See DS3232.h for DS3232 address
//// See MMA845x.h for MMA8452Q address

//// individual pins, used for I2CReset() sequence
//// SCL2/RA2	pin 58	TP34, next to pushbuttons
//// SDA2/RA3 pin 59	TP36, next to batter

//// TRISx (data direction) 1 = input (default), 0 = output
//// ODCx  (open drain control)  1 = open drain, 0 = digital (default)  NOTE: affects ALL modes, including PERIPHERAL control!

//#define I2C2_SCL_TRIS		TRISAbits.TRISA2				// for SmartTrak Hardware
//#define I2C2_SCL_LAT		LATAbits.LATA2

//#define I2C2_SDA_TRIS		TRISAbits.TRISA3				// for SmartTrak Hardware
//#define I2C2_SDA_LAT		LATAbits.LATA3

//#define I2C2_PORT			IOPORT_A
//#define	PIN_I2C2_SCL		BIT_2
//#define	PIN_I2C2_SDA		BIT_3

//#define	I2C_GPIO_SCL_MASK	PIN_I2C2_SCL
//#define	I2C_GPIO_SDA_MASK	PIN_I2C2_SDA
//#define	I2C_GPIO_MASK		(PIN_I2C2_SCL | PIN_I2C2_SDA)


/*
extern inline void __attribute__((always_inline)) InitDiscreteI2C2(void)
{
	// Clock (SCL) and Data (SDA) pins
	PORTClearBits(I2C2_PORT, PIN_I2C2_SCL | PIN_I2C2_SDA);
	PORTSetPinsDigitalIn(I2C2_PORT, PIN_I2C2_SCL | PIN_I2C2_SDA);

}
*/

/*********************************************************************
* IOS FOR ADC Inputs
*********************************************************************/

//#define ADC_TEMP    ADC_CH0_POS_SAMPLEA_AN4
//#define ADC_POT     ADC_CH0_POS_SAMPLEA_AN5


/*********************************************************************
* IOS FOR THE UART
*********************************************************************/

// See SmartTrak.h for UART1, UART2 assignments and baud rates

/*********************************************************************
* RTCC DEFAULT INITIALIZATION (these are values to initialize the RTCC
*********************************************************************/
// these definitions are for the internal RTCC, not currently in use

#define RTCC_DEFAULT_DAY        29      // 29
#define RTCC_DEFAULT_MONTH      4       // April
#define RTCC_DEFAULT_YEAR       13      // 2013
#define RTCC_DEFAULT_WEEKDAY    04      // Thursday
#define RTCC_DEFAULT_HOUR       10      // 10:10:01
#define RTCC_DEFAULT_MINUTE     10
#define RTCC_DEFAULT_SECOND     01

#include <stm32f4xx.h>
/*********************************************************************
* Build & Compile date and time
*********************************************************************/

// Example of __DATE__ string: "Jul 27 2012"
// Example of __TIME__ string: "21:06:19"

#define COMPUTE_BUILD_YEAR \
    ( \
        (__DATE__[ 7] - '0') * 1000 + \
        (__DATE__[ 8] - '0') *  100 + \
        (__DATE__[ 9] - '0') *   10 + \
        (__DATE__[10] - '0') \
    )


#define COMPUTE_BUILD_DAY \
    ( \
        ((__DATE__[4] >= '0') ? (__DATE__[4] - '0') * 10 : 0) + \
        (__DATE__[5] - '0') \
    )


#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')


#define COMPUTE_BUILD_MONTH \
    ( \
        (BUILD_MONTH_IS_JAN) ?  1 : \
        (BUILD_MONTH_IS_FEB) ?  2 : \
        (BUILD_MONTH_IS_MAR) ?  3 : \
        (BUILD_MONTH_IS_APR) ?  4 : \
        (BUILD_MONTH_IS_MAY) ?  5 : \
        (BUILD_MONTH_IS_JUN) ?  6 : \
        (BUILD_MONTH_IS_JUL) ?  7 : \
        (BUILD_MONTH_IS_AUG) ?  8 : \
        (BUILD_MONTH_IS_SEP) ?  9 : \
        (BUILD_MONTH_IS_OCT) ? 10 : \
        (BUILD_MONTH_IS_NOV) ? 11 : \
        (BUILD_MONTH_IS_DEC) ? 12 : \
        /* error default */  99 \
    )

#define COMPUTE_BUILD_HOUR ((__TIME__[0] - '0') * 10 + __TIME__[1] - '0')
#define COMPUTE_BUILD_MIN  ((__TIME__[3] - '0') * 10 + __TIME__[4] - '0')
#define COMPUTE_BUILD_SEC  ((__TIME__[6] - '0') * 10 + __TIME__[7] - '0')


#define BUILD_DATE_IS_BAD (__DATE__[0] == '?')

#define BUILD_YEAR  ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_YEAR)
#define BUILD_MONTH ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_MONTH)
#define BUILD_DAY   ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_DAY)

#define BUILD_TIME_IS_BAD (__TIME__[0] == '?')

#define BUILD_HOUR  ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_HOUR)
#define BUILD_MIN   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_MIN)
#define BUILD_SEC   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_SEC)


/*********************************************************************
* PIC32 SK LEDs
*********************************************************************/

// these functions are for the LEDs on the PIC32 Starter kit board, NOT the PCA9554
#if defined (PLATFORM_PIC32_SK)
	typedef enum
	{
		LED_1,
		LED_2,
		LED_3,
	} PIC32_SK_LED;

	#define PORT_LEDS   IOPORT_D
	#define PIN_LED1    BIT_0
	#define PIN_LED2    BIT_1
	#define PIN_LED3    BIT_2

	extern inline void __attribute__((always_inline)) SetLEDDirection(void)
	{
	   PORTSetPinsDigitalOut(PORT_LEDS, (PIN_LED1 | PIN_LED2 | PIN_LED3));
	}

	extern inline void __attribute__((always_inline)) TurnLEDOn(PIC32_SK_LED led)
	{
		// starter kit LEDs are active HIGH
		if(led == LED_1)
		  PORTSetBits(PORT_LEDS, PIN_LED1);

		if(led == LED_2)
		  PORTSetBits(PORT_LEDS, PIN_LED2);

		if(led == LED_3)
		  PORTSetBits(PORT_LEDS, PIN_LED3);

	}

	extern inline void __attribute__((always_inline)) TurnLEDOff(PIC32_SK_LED led)
	{
		if(led == LED_1)
		  PORTClearBits(PORT_LEDS, PIN_LED1);

		if(led == LED_2)
		  PORTClearBits(PORT_LEDS, PIN_LED2);

		if(led == LED_3)
		  PORTClearBits(PORT_LEDS, PIN_LED3);

	}

	extern inline void __attribute__((always_inline)) ToggleLED(PIC32_SK_LED led)
	{
		if(led == LED_1)
		  PORTToggleBits(PORT_LEDS, PIN_LED1);

		if(led == LED_2)
		  PORTToggleBits(PORT_LEDS, PIN_LED2);

		if(led == LED_3)
		  PORTToggleBits(PORT_LEDS, PIN_LED3);

	}

	extern inline void __attribute__((always_inline)) TurnLEDAllOn(void)
	{
		  PORTSetBits(PORT_LEDS, PIN_LED1);
		  PORTSetBits(PORT_LEDS, PIN_LED2);
		  PORTSetBits(PORT_LEDS, PIN_LED3);

	}

	extern inline void __attribute__((always_inline)) TurnLEDAllOff(void)
	{
		  PORTClearBits(PORT_LEDS, PIN_LED1);
		  PORTClearBits(PORT_LEDS, PIN_LED2);
		  PORTClearBits(PORT_LEDS, PIN_LED3);
	}

#elif defined (PLATFORM_SMARTTRAK_V1)
	// SmartTrak hardware places all controllable LEDs on I/O Expander U9
	//	U9		UCA9554		I/O Expander for LEDs (12C)		RA2 (SCL), RA3 (SDA)

	typedef enum
	{
		LED_1,
		LED_2,
		LED_3,
		LED_4,
		LED_5,
		LED_6,
		LED_7,
		LED_8,

	} SMARTTRAK_LED;

	extern inline void __attribute__((always_inline)) SetLEDDirection(void)
	{

	}

	extern inline void __attribute__((always_inline)) TurnLEDOn(SMARTTRAK_LED led)
	{

	}

	extern inline void __attribute__((always_inline)) TurnLEDOff(SMARTTRAK_LED led)
	{

	}

	extern inline void __attribute__((always_inline)) ToggleLED(SMARTTRAK_LED led)
	{

	}

	extern inline void __attribute__((always_inline)) TurnLEDAllOn(void)
	{

	}

	extern inline void __attribute__((always_inline)) TurnLEDAllOff(void)
	{

	}

	//	U9		UCA9554		I/O Expander for LEDs (12C)		RA2 (SCL), RA3 (SDA)
#endif		// NOT PLATFORM_PIC32_SK

#if defined (PLATFORM_SMARTTRAK_V1) && defined (DISPLAY_XC32_MESSAGES)
    #warning  "SmartTrak hardware LEDs not yet implemented"
#endif


/*********************************************************************
* PIC32 SK Switches
*********************************************************************/

#if defined (PLATFORM_PIC32_SK)
	typedef enum
	{
		SWITCH_1,
		SWITCH_2,
		SWITCH_3,
	} PIC32_SK_SWITCHES;

	#define PORT_SWITCHES   IOPORT_D
	#define PIN_SW1    BIT_6
	#define PIN_SW2    BIT_7
	#define PIN_SW3    BIT_13

	#define BIT_SW1    RD6
	#define BIT_SW2    RD7
	#define BIT_SW3    RD13

#elif defined (PLATFORM_SMARTTRAK_V1)
	//	U8		UCA9554		I/O Expander for Switches(I2C)	RA2 (SCL), RA3 (SDA), RE8 (INT)
#endif		// NOT PLATFORM_PIC32_SK

#if defined (PLATFORM_SMARTTRAK_V1) && defined (DISPLAY_XC32_MESSAGES)
    #warning  "SmartTrak hardware Switches not yet implemented"
#endif


/*********************************************************************
* PIC32 Motor Drive Definitions
*********************************************************************/

#if defined (PLATFORM_PIC32_SK)

	// NOTE: PIC32 Starter Kit wiring is VERY different from the SmartTrak wiring,
	//		and several pins are simply jumpered rather than controlled by the MCU

	//	SLEW	default (open) low = SLOW
	//	INV		default (open) low = NON INVERTED
	//	EN		default (open) low = SLEEP, must be driven HIGH by MCU
	//	D1		default (open) high = DISABLED, must be driven LOW by jumper
	// /D2		default (open) low = DISABLED, must be driven HIGH by jumper

	// Connections to MC33926 H-Bridge ICs
	//	Motor Enable EN			RE4			Digital Out

	//	Bridge1 Feedback FB		RB4/AN4		Analog In
	//	Bridge1 Status Flag /SF	RB2			Digital In
	// 	Bridge1 PWM_1			OC1/RD0		Peripheral Out
	// 	Bridge1 PWM_2			OC2/RD1		Peripheral Out
	//	Bridge1 SLEW			open, LOW
	//	Bridge1 INV				open, LOW
	//	Bridge1 D1				jumper LOW
	//	Bridge1 /D2				jumper HIGH

	//	Bridge2 Feedback FB		RB5/AN5		Analog In
	//	Bridge2 Status Flag /SF	RB3			Digital In
	// 	Bridge2 PWM_1			OC3/RD2		Peripheral Out
	// 	Bridge2 PWM_2			OC4/RD3		Peripheral Out
	//	Bridge2 SLEW			open, LOW
	//	Bridge2 INV				open, LOW
	//	Bridge2 D1				jumper LOW
	//	Bridge2 /D2				jumper HIGH

	#define PORT_BRIDGE_ENABLE	IOPORT_E
	#define	PIN_BRIDGE_ENABLE	BIT_4

	#define PORT_BRIDGE_CTRL	IOPORT_B
	#define PIN_BRIDGE1_FB		BIT_4
	#define PIN_BRIDGE1_SF		BIT_2

	#define PIN_BRIDGE2_FB		BIT_5
	#define PIN_BRIDGE2_SF		BIT_3

	extern inline void __attribute__((always_inline)) InitMotorDrive(void)
	{
		// Feedback (FB) and Status Flag (SF) pins
		PORTClearBits(PORT_BRIDGE_CTRL, PIN_BRIDGE1_FB | PIN_BRIDGE1_SF | PIN_BRIDGE2_FB | PIN_BRIDGE2_SF);
		PORTSetPinsDigitalIn(PORT_BRIDGE_CTRL, PIN_BRIDGE1_SF | PIN_BRIDGE2_SF);

		// set Feedback pins to Analog IN

		// Enable Pins, initially LOW
		PORTClearBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE_ENABLE);
		PORTSetPinsDigitalOut(PORT_BRIDGE_ENABLE, PIN_BRIDGE_ENABLE);
	}

	extern inline void __attribute__((always_inline)) EnableMotorDrive(void)
	{
		  PORTSetBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE_ENABLE);
	}

	extern inline void __attribute__((always_inline)) DisableMotorDrive(void)
	{
		  PORTClearBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE_ENABLE);
	}

	extern inline UINT16 __attribute__((always_inline)) ReadBridgeSFPins(void)
	{
		return (PORTReadBits(PORT_BRIDGE_CTRL, PIN_BRIDGE1_SF | PIN_BRIDGE2_SF));
	}

#elif defined (PLATFORM_SMARTTRAK_V1) && defined (DRIV_MC33926)
	//	SmartTrak Hardware MCU Connections
	//	U1		MC33926		Azimuth H-Bridge				RB2 (/SF), OC2 (IN2), OC1(IN1), RE1 (/D2), RE0(D1), RF0 (EN), RG3 (SLEW), RG?? (INV), RB4 (FB)
	//	U2		MC33926		Elevation H-Bridge				RB3 (/SF), OC4 (IN2), OC3(IN1), RE3 (/D2), RE2(D1), RF1 (EN), RE5 (SLEW), RE6 (INV), RB5 (FB)

	// Connections to MC33926 H-Bridge ICs

	//	Bridge1 Enable EN		RF0			Digital Out		split, different port
	//	Bridge1 Feedback FB		RB4/AN4		Analog In		same
	//	Bridge1 Status Flag /SF	RB2			Digital In		same
	// 	Bridge1 PWM_1			OC1/RD0		Peripheral Out	same
	// 	Bridge1 PWM_2			OC2/RD1		Peripheral Out	same
	//	Bridge1 SLEW			RG3			Digital Out, LOW
	//	Bridge1 INV				RG2			Digital Out, LOW
	//	Bridge1 D1				RE0			Digital Out, LOW
	//	Bridge1 /D2				RE1			Digital Out, HIGH

	//	Bridge2 Enable EN		RF1			Digital Out
	//	Bridge2 Feedback FB		RB5/AN5		Analog In
	//	Bridge2 Status Flag /SF	RB3			Digital In
	// 	Bridge2 PWM_1			OC3/RD2		Peripheral Out
	// 	Bridge2 PWM_2			OC4/RD3		Peripheral Out
	//	Bridge2 SLEW			RE5			Digital Out, LOW
	//	Bridge2 INV				RE6			Digital Out, LOW
	//	Bridge2 D1				RE2			Digital Out, LOW
	//	Bridge2 /D2				RE3			Digital Out, HIGH

	#define PORT_BRIDGE_ENABLE	IOPORT_F
	#define	PIN_BRIDGE1_ENABLE	BIT_0
	#define	PIN_BRIDGE2_ENABLE	BIT_1


	#define PORT_BRIDGE_CTRL	IOPORT_B
	#define PIN_BRIDGE1_FB		BIT_4
	#define PIN_BRIDGE1_SF		BIT_2

	#define PIN_BRIDGE2_FB		BIT_5
	#define PIN_BRIDGE2_SF		BIT_3

	#define PORT_BRIDGE1_CONFIG	IOPORT_G
	#define PIN_BRIDGE1_SLEW	BIT_3
	#define PIN_BRIDGE1_INV		BIT_2

	#define PORT_BRIDGE2_CONFIG	IOPORT_E
	#define PIN_BRIDGE2_SLEW	BIT_5
	#define PIN_BRIDGE2_INV		BIT_6


	#define PORT_BRIDGE_DISABLE	IOPORT_E
	#define	PIN_BRIDGE1_D1		BIT_0
	#define	PIN_BRIDGE1_NOT_D2	BIT_1

	#define	PIN_BRIDGE2_D1		BIT_2
	#define	PIN_BRIDGE2_NOT_D2	BIT_3

	extern inline void __attribute__((always_inline)) InitMotorDrive(void)
	{
		// Feedback (FB) and Status Flag (SF) pins
		PORTClearBits(PORT_BRIDGE_CTRL, PIN_BRIDGE1_FB | PIN_BRIDGE1_SF | PIN_BRIDGE2_FB | PIN_BRIDGE2_SF);
		PORTSetPinsDigitalIn(PORT_BRIDGE_CTRL, PIN_BRIDGE1_SF | PIN_BRIDGE2_SF);

		// set Feedback pins to Analog IN


		// Configuration Bits SLEW and INV, both LOW
		PORTClearBits(PORT_BRIDGE1_CONFIG, PIN_BRIDGE1_SLEW | PIN_BRIDGE1_INV);
		PORTSetPinsDigitalOut(PORT_BRIDGE1_CONFIG, PIN_BRIDGE1_SLEW | PIN_BRIDGE1_INV);
		PORTClearBits(PORT_BRIDGE2_CONFIG, PIN_BRIDGE2_SLEW | PIN_BRIDGE2_INV);
		PORTSetPinsDigitalOut(PORT_BRIDGE2_CONFIG, PIN_BRIDGE2_SLEW | PIN_BRIDGE2_INV);

		// Disable bits, initialze to DISABLED, D1 HIGH, /D2 LOW
		PORTSetBits(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_D1 | PIN_BRIDGE2_D1);
		PORTClearBits(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_NOT_D2 | PIN_BRIDGE2_NOT_D2);
		PORTSetPinsDigitalOut(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_D1 | PIN_BRIDGE1_NOT_D2 | PIN_BRIDGE2_D1 | PIN_BRIDGE2_NOT_D2);

		// Enable Pins, initially LOW
		PORTClearBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);
		PORTSetPinsDigitalOut(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);
	}

	extern inline void __attribute__((always_inline)) EnableMotorDrive(void)
	{
		// Disable bits, to ENABLED, D1 LOW, /D2 HIGH
		PORTClearBits(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_D1 | PIN_BRIDGE2_D1);
		PORTSetBits(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_NOT_D2 | PIN_BRIDGE2_NOT_D2);

		PORTSetBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);
	}

	extern inline void __attribute__((always_inline)) DisableMotorDrive(void)
	{
		PORTClearBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);

		// Disable bits, to DISABLED, D1 HIGH, /D2 LOW
		PORTSetBits(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_D1 | PIN_BRIDGE2_D1);
		PORTClearBits(PORT_BRIDGE_DISABLE, PIN_BRIDGE1_NOT_D2 | PIN_BRIDGE2_NOT_D2);
	}

	extern inline UINT16 __attribute__((always_inline)) ReadBridgeSFPins(void)
	{
		return (PORTReadBits(PORT_BRIDGE_CTRL, PIN_BRIDGE1_SF | PIN_BRIDGE2_SF));
	}

#elif defined (PLATFORM_SMARTTRAK_V1) && defined (DRIV_VNH5050A)
        //	SmartTrak Hardware MCU Connections
	//	U1		VNH5050A-E		Azimuth H-Bridge	AN1 (CS), OC2 (INB), OC1(INA), RF0 (EN), OC5(IN_PWM)
	//	U2		VNH5050A-E		Elevation H-Bridge	AN2 (CS), OC4 (INB), OC3(INA), RF1 (EN), OC5(IN_PWM)

	// Connections to VNH5050A-E H-Bridge ICs

	//	Bridge1 Enable EN		RF0			Digital Out		split, different port
	// 	Bridge1 PWM_1			OC1/RD0		Peripheral Out	same
	// 	Bridge1 PWM_2			OC2/RD1		Peripheral Out	same
        // 	Bridge1 IN_PWM			OC5		Peripheral Out	same

	//	Bridge2 Enable EN		RF1			Digital Out
	// 	Bridge2 PWM_1			OC3/RD2		Peripheral Out
	// 	Bridge2 PWM_2			OC4/RD3		Peripheral Out
          // 	Bridge1 IN_PWM			OC5		Peripheral Out	same

	#define PORT_BRIDGE_ENABLE	IOPORT_F
	#define	PIN_BRIDGE1_ENABLE	BIT_0
	#define	PIN_BRIDGE2_ENABLE	BIT_1
 
	extern inline void __attribute__((always_inline)) InitMotorDrive(void)
	{
		PORTClearBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);
		PORTSetPinsDigitalOut(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);
	}

	extern inline void __attribute__((always_inline)) EnableMotorDrive(void)
	{
		

		PORTSetBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);
	}

	extern inline void __attribute__((always_inline)) DisableMotorDrive(void)
	{
		PORTClearBits(PORT_BRIDGE_ENABLE, PIN_BRIDGE1_ENABLE | PIN_BRIDGE2_ENABLE);

	
	}

	extern inline UINT16 __attribute__((always_inline)) ReadBridgeSFPins(void)
	{
		//???
	}

#endif		// NOT PLATFORM_PIC32_SK


/*********************************************************************
* PIC32 Motion Sensor Definitions
*********************************************************************/

#if defined (PLATFORM_PIC32_SK)
	// Pin to Input Capture Module Assignments:
	// RD8		IC1
	// RD9		IC2
	// RD10		IC3
	// RD11		IC4

	//	Azimuth Motion Sensor	IC1		RD8		Digital In	IC1
	//	Elevation Motion Sensor	IC2		RD9		Digital In	IC2

	#define PORT_INPUT_CAPTURE		IOPORT_D
	#define PORT_MOTION_SENSOR		PORT_INPUT_CAPTURE

	#define	PIN_AZIMUTH_SENSOR		BIT_8
	#define	PIN_ELEVATION_SENSOR	BIT_9

	// these functions allow direct reading of the pins; they are NOT necessary for Input Capture
	extern inline void __attribute__((always_inline)) InitMotionSensorPins(void)
	{
		PORTClearBits(PORT_MOTION_SENSOR, PIN_AZIMUTH_SENSOR | PIN_ELEVATION_SENSOR );

		PORTSetPinsDigitalIn(PORT_MOTION_SENSOR, PIN_AZIMUTH_SENSOR | PIN_ELEVATION_SENSOR );
	}

	extern inline UINT16 __attribute__((always_inline)) ReadMotionSensorPins(void)
	{
		return (PORTReadBits(PORT_MOTION_SENSOR, PIN_AZIMUTH_SENSOR | PIN_ELEVATION_SENSOR ));
	}

	// definitions for Input Capture
	#define	AZ_ICCONbits		IC1CONbits
	#define	AZ_ICBUF			IC1BUF
	#define	AZ_ICClearIntFlag	mIC1ClearIntFlag
	#define	AZ_OpenCapture		OpenCapture1
	#define	AZ_ConfigIntCapture	ConfigIntCapture1

	#define	EL_ICCONbits		IC2CONbits
	#define	EL_ICBUF			IC2BUF
	#define	EL_ICClearIntFlag	mIC2ClearIntFlag
	#define	EL_OpenCapture		OpenCapture2
	#define	EL_ConfigIntCapture	ConfigIntCapture2

#elif defined (PLATFORM_SMARTTRAK_V1)
	// Pin to Input Capture Module Assignments:
	// RD8		IC1
	// RD9		IC2
	// RD10		IC3
	// RD11		IC4

	//	AZ0		Azimuth Hall Effect A		RD10		Digital In	IC3
	//	AZ1		Azimuth Hall Effect B		RD11		Unused
	//	EL0		Elevation Hall Effect A		RD8			Digital In	IC1
	//	EL1		Elevation Hall Effect B		RD9			Unused

	#define PORT_INPUT_CAPTURE		IOPORT_D
	#define PORT_MOTION_SENSOR		PORT_INPUT_CAPTURE

	#define	PIN_AZIMUTH_SENSOR		BIT_10
	#define	PIN_ELEVATION_SENSOR	BIT_8

	// these functions allow direct reading of the pins; they are NOT necessary for Input Capture
	extern inline void __attribute__((always_inline)) InitMotionSensorPins(void)
	{
		PORTClearBits(PORT_MOTION_SENSOR, PIN_AZIMUTH_SENSOR | PIN_ELEVATION_SENSOR );

		PORTSetPinsDigitalIn(PORT_MOTION_SENSOR, PIN_AZIMUTH_SENSOR | PIN_ELEVATION_SENSOR );
	}

	extern inline UINT16 __attribute__((always_inline)) ReadMotionSensorPins(void)
	{
		return (PORTReadBits(PORT_MOTION_SENSOR, PIN_AZIMUTH_SENSOR | PIN_ELEVATION_SENSOR ));
	}

	// definitions for Input Capture
	#define	AZ_ICCONbits		IC3CONbits
	#define	AZ_ICBUF			IC3BUF
	#define	AZ_ICClearIntFlag	mIC3ClearIntFlag
	#define	AZ_OpenCapture		OpenCapture3
	#define	AZ_ConfigIntCapture	ConfigIntCapture3

	#define	EL_ICCONbits		IC1CONbits
	#define	EL_ICBUF			IC1BUF
	#define	EL_ICClearIntFlag	mIC1ClearIntFlag
	#define	EL_OpenCapture		OpenCapture1
	#define	EL_ConfigIntCapture	ConfigIntCapture1

#endif		// PLATFORM_SMARTTRAK_V1

/*********************************************************************
* PIC32 Motion Sensor Feedback Simulator Definitions
*********************************************************************/
#ifdef	USE_FEEDBACK_SIMULATOR

	#if defined (PLATFORM_PIC32_SK)

		//	Azimuth Motion Feedback Simulator	RF0		Digital Out
		//	Elevation Motion Feedback Simulator	RF1		Digital Out

		#define PORT_FEEDBACK_SIMULATOR	IOPORT_F

		#define	PIN_AZIMUTH_FEEDBACK_SIMULATOR		BIT_0
		#define	PIN_ELEVATION_FEEDBACK_SIMULATOR	BIT_1

	extern inline void __attribute__((always_inline)) InitMotionFeedbackSimulatorPins(void)
	{
		PORTClearBits(PORT_FEEDBACK_SIMULATOR, PIN_AZIMUTH_FEEDBACK_SIMULATOR | PIN_ELEVATION_FEEDBACK_SIMULATOR );

		PORTSetPinsDigitalOut(PORT_FEEDBACK_SIMULATOR, PIN_AZIMUTH_FEEDBACK_SIMULATOR | PIN_ELEVATION_FEEDBACK_SIMULATOR );
	}

	extern inline void __attribute__((always_inline)) SetFeedbackSimulator(UINT16 nAxis)
	{
		if (nAxis == 0)
			PORTSetBits(PORT_FEEDBACK_SIMULATOR, PIN_AZIMUTH_FEEDBACK_SIMULATOR);
		else if (nAxis == 1)
			PORTSetBits(PORT_FEEDBACK_SIMULATOR, PIN_ELEVATION_FEEDBACK_SIMULATOR);
	}

	extern inline void __attribute__((always_inline)) ClearFeedbackSimulator(UINT16 nAxis)
	{
		if (nAxis == 0)
			PORTClearBits(PORT_FEEDBACK_SIMULATOR, PIN_AZIMUTH_FEEDBACK_SIMULATOR);
		else if (nAxis == 1)
			PORTClearBits(PORT_FEEDBACK_SIMULATOR, PIN_ELEVATION_FEEDBACK_SIMULATOR);
	}

	#elif defined (PLATFORM_SMARTTRAK_V1)
		#error  "SmartTrak hardware version not yet defined"
	#endif		// NOT PLATFORM_PIC32_SK

#endif		// USE_FEEDBACK_SIMULATOR


/*********************************************************************
* PIC32 Oscilloscope Trigger Definitions
*********************************************************************/

// assigned pins are the same for both platforms

#define PORT_SCOPE_TRIGGER					IOPORT_B

#define	PIN_SCOPE_TRIGGER1					BIT_10				// PIC32 Starter Kit I/O Expansion J10-47, SmartTrak J10-9
#define	PIN_SCOPE_TRIGGER2					BIT_11


extern inline void __attribute__((always_inline)) InitTriggerPins(void)
{
	PORTClearBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1 | PIN_SCOPE_TRIGGER2 );

	PORTSetPinsDigitalOut(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1 | PIN_SCOPE_TRIGGER2 );
}

extern inline void __attribute__((always_inline)) Trigger1Level(UINT8 cLevel)
{
	if (cLevel == 0)
	{
		PORTClearBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
	}
	else if (cLevel == 1)
	{
		PORTClearBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
		PORTSetBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
	}
}

extern inline void __attribute__((always_inline)) Trigger1Pulse(void)
{
	PORTClearBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
	PORTSetBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
	PORTClearBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
}


extern inline void __attribute__((always_inline)) Trigger1Toggle(void)
{
	if (PORTReadBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1) == 0)
		PORTSetBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
	else
		PORTClearBits(PORT_SCOPE_TRIGGER, PIN_SCOPE_TRIGGER1);
}



//#define	TOGGLE_DEBUG_PIN1	DEBUG_PIN1 = ((DEBUG_PIN1 == 1) ? 0 : 1)
//#define	DEBUG_PIN2_HI		DEBUG_PIN2 = 0; DEBUG_PIN2 = 1
//#define	DEBUG_PIN2_LOW		DEBUG_PIN2 = 0;
//#define	DEBUG_PIN3_HI		DEBUG_PIN3 = 0; DEBUG_PIN3 = 1
//#define	DEBUG_PIN3_LOW		DEBUG_PIN3 = 0;


/*********************************************************************
* J9 Spare Pins
*********************************************************************/

// Note: J9-1 and J9-2 are assigned above as scope trigger pins, but could be reassigned here
// assigned pins are the same for both platforms (?)

#define PORT_J9								IOPORT_B

#define	PIN_J9_3							BIT_12
#define	PIN_J9_4							BIT_13
#define	PIN_J9_5							BIT_14
#define	PIN_J9_6							BIT_15

// pin usage assignments
#define	PIN_XBEE_RESET						PIN_J9_3
#define	PIN_XBEE_DETECT						PIN_J9_4		// input

#define	PIN_MOTOR_FWD_RELAY					PIN_J9_5
#define	PIN_MOTOR_REV_RELAY					PIN_J9_6



extern inline void __attribute__((always_inline)) InitJ9Pins(void)
{
	PORTSetBits(PORT_J9, PIN_J9_3 | PIN_J9_4 | PIN_J9_5 | PIN_J9_6 );			// relay drive for 2 x 5V Relay board is active LOW, XBee Reset is active LOW

	PORTSetPinsDigitalOut(PORT_J9, PIN_J9_3 | PIN_J9_5 | PIN_J9_6 );

	PORTSetPinsDigitalIn(PORT_J9, PIN_J9_4);

}

extern inline void __attribute__((always_inline)) SetJ9Pin(UINT16 J9_Pin)
{
	PORTSetBits(PORT_J9, J9_Pin);
}

extern inline void __attribute__((always_inline)) ClearJ9Pin(UINT16 J9_Pin)
{
	PORTClearBits(PORT_J9, J9_Pin);
}


extern inline UINT16 __attribute__((always_inline)) ReadJ9Pin(UINT16 J9_Pin)
{
	return (PORTReadBits(PORT_J9, J9_Pin));
}


#endif // __HARDWARE_PROFILE_H
