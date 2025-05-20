// *************************************************************************************************
//										A D C 1 0 . C
// *************************************************************************************************
//
//		Project:	String Monitoring
//
//		Contains:	PIC32 ADC10 Functions
//
//
// *************************************************************************************************

// These functions are NOT register level. They call into the PIC32 Peripheral library to
// handle most register access, especially initialization
/*
 * Platform: Explorer-16 with PIC32MX PIM
 *
 * Features demonstrated:
 *    - ADC setup and configuration using the peripheral library
 *    - Reading a potentiometer and temperature sensor with the
 *      ADC10 module
 *
 * Description: 
 *      In this example two channels are alternately sampled (the pot
 *      and temperature sensor) with the ADC operating in Alternate 
 *      AutoSample mode. The most current results can be read from the
 *      pot and tempSensor variables.
 * 
 * Notes:
 *    - PIC32MX 2xx PIMS are unconnected to the potentiometer and
 *      temperature sensor on the Explorer-16. The README states the
 *      soldering instructions that must be followed for proper
 *      functionality in this code example.
 ********************************************************************/

#ifndef __32MX360F512L__
	#error	Incorrect Processor Type Selected; code is for PIC32MX360F512L
#endif

//-----------------------------------------------------------------------------
//								#include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>
#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>						// Microchip PIC32 peripheral library main header
//lint +e14

#include <legacy\int_3xx_4xx_legacy.h>	// required for various interrupt handlers

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"
#include "ADC10Read.h"


#ifdef NOTDEF
	// for reference only, see MCUConfigurationBits.h for project implementation
	//#if defined (__32MX360F512L__)
	// Configuration Bit settings
	// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
	// PBCLK = 80 MHz (SYSCLK / FPBDIV)
	// Primary Osc w/PLL (XT+,HS+,EC+PLL)
	// WDT OFF
	// Other options are don't care
	#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
	#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
	#define SYS_FREQ (80000000L)
#endif

// To configure the ADC module, perform the following steps:
// 1. Configure the analog port pins in AD1PCFG<15:0> (see 17.4.1).
// 2. Select the analog inputs to the ADC multiplexers in AD1CHS<32:0> (see 17.4.2).
// 3. Select the format of the ADC result using FORM<2:0> (AD1CON1<10:8>) (see 17.4.3).
// 4. Select the sample clock source using SSRC<2:0> (AD1CON1<7:5>) (see 17.4.4).
// 5. Select the voltage reference source using VCFG<2:0> (AD1CON2<15:13>) (see 17.4.7).
// 6. Select the Scan mode using CSCNA (AD1CON2<10>) (see 17.4.8).
// 7. Set the number of conversions per interrupt SMP<3:0> (AD1CON2<5:2>), if interrupts are to be used (see 17.4.9).
// 8. Set Buffer Fill mode using BUFM (AD1CON2<1>) (see 17.4.10).
// 9. Select the MUX to be connected to the ADC in ALTS AD1CON2<0> (see 17.4.11).
// 10. Select the ADC clock source using ADRC (AD1CON3<15>) (see 17.4.12).
// 11. Select the sample time using SAMC<4:0> (AD1CON3<12:8>), if auto-convert is to be used (see 17-2).
// 12. Select the ADC clock prescaler using ADCS<7:0> (AD1CON3<7:0>) (see 17.4.12).
// 13. Turn the ADC module on using AD1CON1<15> (see 17.4.14).


// 14. To configure ADC interrupt (if required):
//		a) Clear the AD1IF bit (IFS1<1>) (see 17.7).
//		b) Select ADC interrupt priority AD1IP<2:0>(IPC<28:26>) and subpriority AD1IS<1:0> (IPC<24:24>) if interrupts are to be used (see 17.7).
// 15. Start the conversion sequence by initiating sampling (see 17.4.15).// Define setup parameters for OpenADC10 function

// define setup parameters for OpenADC10

				// Output in integer format | Trigger mode auto | Enable autosample
#define config1		ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

					// ADC ref external | Disable offset test | Enable scan mode | Perform 12 samples | 16 word buffer | Don't Use alternate mode
#define config2     ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_14 | ADC_BUF_16 | ADC_ALT_INPUT_OFF
					// Use ADC internal clock | Set sample time  (maximum sample time)
#define config3     ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_16 | ADC_CONV_CLK_3Tcy2//ADC_CONV_CLK_Tcy//ADC_CONV_CLK_63Tcy2
			// Skip AN0, AN1 (pins are used for debug interface!)
#define configscan  SKIP_SCAN_AN6| SKIP_SCAN_AN7| SKIP_SCAN_AN8|SKIP_SCAN_AN9| SKIP_SCAN_AN10 | SKIP_SCAN_AN11| SKIP_SCAN_AN12| SKIP_SCAN_AN13| SKIP_SCAN_AN14| SKIP_SCAN_AN15

// Enable 4 channels for analog input - AN1,AN2,AN5,AN9 (0,1,2,....)
//#define configport   ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN5_ANA | ENABLE_AN9_ANA
#define configport   ENABLE_AN0_ANA | ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN3_ANA | ENABLE_AN4_ANA | ENABLE_AN5_ANA


void InitADC10(void)
{
    
    CloseADC10();   // Ensure the ADC is off before setting the configuration

    // Configure to sample AN4 & AN5
				// Use ground as neg ref for A
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);

    // Configure ADC using the parameters defined above
    OpenADC10( config1, config2, config3, configport, configscan);

    EnableADC10(); // Enable the ADC

}


int ReadADC10Value(UINT8 bADCChannel)
{
	int nReading;

	while ( !mAD1GetIntFlag() )
	{
		// Wait for the first conversion to complete so there will be vaild data in ADC result registers
	}

	// Read the result of temperature sensor conversion from the idle buffer
	nReading = ReadADC10(bADCChannel);

	mAD1ClearIntFlag();

	return(nReading);

}


