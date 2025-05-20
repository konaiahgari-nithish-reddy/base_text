// *************************************************************************************************
//							M C U C o n f i g u r a t i o n B i t s . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	PIC32MX360 Configuration Bits
//
// *************************************************************************************************
// *******************************************************************************

// References:
//	PIC32MX3XX-4XX Family Data Sheet - Microchip 61143H.pdf, 26.0 SPECIAL FEATURES, Page 131
//	C:\Program Files\Microchip\xc32\v1.20\docs\hlpPIC32MXConfigSet.chm

// *****************************************************************************
// Section: Configuration bits
// *****************************************************************************
// *****************************************************************************

// Oscillator Setup
#pragma config POSCMOD  = XT            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FCKSM = CSECME			// Clock Switching & Fail Safe Clock Monitor
//#pragma config FCKSM    = CSDCMD
#pragma config OSCIOFNC = ON			// CLKO Enable
//#pragma config OSCIOFNC = OFF
#pragma config FSOSCEN = ON
//#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable

// PLLS
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (PIC32 Starter Kit: use divide by 2 only)
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_8         // Peripheral Clock divisor (was DIV_2)

// Watchdog
#pragma config FWDTEN   = OFF           // Watchdog Timer (software enable at run time)
#pragma config WDTPS    = PS2048		// Watchdog Timer Postscale for 65.6mS watchdog timeout
#pragma config CP       = OFF           // Code Protect

// Write Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect

// Debugger
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = OFF           // Debugger Disabled for Starter Kit

// end of MCUConfigurationBits.h
