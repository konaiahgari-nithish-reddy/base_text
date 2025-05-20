/*******************************************************************
 *
 *	Hardware Platform specific definitions
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC32
 * Compiler:        Microchip XC32 1.20 or higher
 *
 * created by gsf Engineering for SmartTrak
 *
 * Author: Steve Kranish
 *          gsf Engineering, Beverly MA USA
 *          skranish@verizon.net
 *          978-927-7189
 *
 * 17 Feb 13 <sek> changed platform definitions to avoid use of PIC32_STARTER_KIT macro
 *
 ********************************************************************/
// 13 Feb 13 <sek> reduced from Microchip file to SmartTrak project requirements

#ifndef __PIC32MX__
    #error  "Firmware source code ONLY supports PIC32"
#endif


#if defined (PLATFORM_PIC32_SK)
	
    /*********************************************************************
     * Hardware Configuration for 
     * PIC32 Starter Kit
     * Starter Kit I/O Expansion board
     * PICTail+ Serial Memory Board
     * PICTail+ Prototype board containing I2C devices
     *
     ********************************************************************/
    #include "BSP\HardwareProfile_PIC32_STK.h"

#elif defined (PLATFORM_SMARTTRAK_V1)
    #include "BSP\HardwareProfile_PIC32_STK.h"
#endif

    
