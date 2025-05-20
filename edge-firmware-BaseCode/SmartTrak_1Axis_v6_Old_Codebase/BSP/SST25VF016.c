/*****************************************************************************
 *
 * Basic access to SPI Flash SST25VF016 located on 
 * Graphics LCD Controller PICtail Plus SSD1926 Board.
 *
 *****************************************************************************
 * FileName:        SST25VF016.c
 * Dependencies:    SST25VF016.h
 * Processor:       PIC24F, PIC24H, dsPIC, PIC32
 * Compiler:       	MPLAB C30 V3.00, MPLAB C32
 * Linker:          MPLAB LINK30, MPLAB LINK32
 * Company:         Microchip Technology Incorporated
 *
 * Software License Agreement
 *
 * Copyright � 2008 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).  
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Anton Alkhimenok		01/07/09	...
 * 12 Feb 13 <sek> modified SST25Init() for PICTail+ SPI Flash board with SST25VF016, cooments, removed unused code
 * 26 Feb 13 <sek> SST25ReadID()
 * 25 Mar 13 <sek> #include "config.h"
 * 17 Aug 13 <sek> #include "TimeDelay.h"			// for DelayMs()
 * 21 Aug 13 <sek> added delay, then test for busy to SPIPut()

 *****************************************************************************/
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"				// gsf standard #defines
#include "HardwareProfile.h"

#include "TimeDelay.h"
#include "SST25VF016.h"

#ifndef __PIC32MX__
	#error	"SST25VF016.C only supports PIC32MX SPI Implementation"
#endif


/************************************************************************
* Function: SST25Init                                                  
*                                                                       
* Overview: this function setup SPI and IOs connected to SST25
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST25Init(void)
{

    /************************************************************************
    * For Explorer 16 RD12 is connected to EEPROM chip select.
    * To prevent a conflict between this EEPROM and SST25 flash
    * RD12 should be pulled up.
    ************************************************************************/
    // Initialize SPI
	SST25_SPISTAT = 0;				// clear all status bits (not all can be written)
	SST25_SPIBRG = 0;				// clear Baud Rate Divisor bits

	SST25_SPICON = 0;				// initialize all SPI1CON bits to 0, no framed support
									// MCLKSEL:Master Clock Select bit, 0= PBCLK is used by the Baud Rate Generator
									// ENHBUF: Enhanced Buffer Enable bit, 0= Enhanced Buffer mode is disabled
									// SIDL: Stop in Idle Mode bit, 0= Continue operation in Idle mode
									// DISSDO: Disable SDOx pin bit, 0= SDOx pin is controlled by the module
									// MODE<32,16>: 32/16-bit Communication Select bits, there is no AUDEN bit in this MCU, so assume 0
									//		When AUDEN = 0, 0 : 8-bit
									// DISSDI: Disable SDI bit, 0= SDIx pin is controlled by the SPI module

	SST25_SPICONbits.MSTEN = 1;		// Master Mode Enable bit, 1 = Master mode
	SST25_SPICONbits.CKP = 0;		// Clock Polarity Select bit, 0 = Idle state for clock is a low level; active state is a high level
	SST25_SPICONbits.CKE = 1;		// SPI Clock Edge Select bit, 1 = Serial output data changes on transition from active clock state to idle clock state (see CKP bit)
	SST25_SPICONbits.SMP = 1;		// SPI Data Input Sample Phase bit, Master mode (MSTEN = 1): 1= Input data sampled at end of data output time

	SST25_SPIBRG = 1;				// bit 12-0 BRG<12:0>: Baud Rate Divisor bits
	SST25_SPICONbits.ON = 1;		// ON: SPI Peripheral On bit, 1= SPI Peripheral is enabled


	// Set IOs directions for SST25 SPI
    SST25_CS_LAT = 1;					// /CS is initially NOT selected
    SST25_CS_TRIS = 0;					// set /CS to OUTPUT

	// /HOLD, change RA6 to RF1
    SST25_HOLD_LAT = 1;					// /HOLD is initially NOT selected
    SST25_HOLD_TRIS = 0;				// set /HOLD to OUTPUT

    SST25ResetWriteProtection();
}

/************************************************************************
* Macros SPIGet()
*
* Overview:  this macros gets a byte from SPI
*
* Input: none
*
* Output: none
*
************************************************************************/
#define SPIGet()     SST25_SPIBUF

/************************************************************************
* Function SPIPut(BYTE data)
*
* Overview:  this function sends a byte
*
* Input: byte to be sent
*
* Output: none
*
************************************************************************/
void SPIPut(BYTE data)
{

//    BYTE    temp;

    // Wait for free buffer in MCU SPI channel
    while(!SST25_SPISTATbits.SPITBE)
		;
    SST25_SPIBUF = data;

    // Wait for data byte to be transferred from MCU SPI channel
    while(!SST25_SPISTATbits.SPIRBF)
		;

	// <sek> problem here is that checking the MCU SPI registers after a write is largely meaningless
	// we REALLY need to check the SPI Flash Status register for
	Delay10us(2L);						// <sek> 21 Aug 13 timing kludge

#ifdef DOES_NOT_WORK_CORRECTLY
	// check for FLASH Memory busy with write
	// this does NOT work correctly, it appears to cause the system to hang during DATA WRITEs
	do
	{
		SPIPut(SST25_CMD_RD_STATUS_REG);		// <sek> was SST25_CMD_RDSR
		SPIGet();

		SPIPut(0);
		temp = SPIGet();
		SST25CSHigh();
	} while (temp & 0x01);
#endif

}



/************************************************************************
* Function: SST25ReadID()
*
* Overview: this function reads JEDEC ID, Manufacturer, Device Type
*
* Input: none
*
* Output: TRUE is as expected, otherwise FALSE

* See SST25VF016 datasheet Page 20
*
************************************************************************/
BOOL SST25ReadID(BYTE *pData)
{
	BOOL	bRetVal = TRUE;
    SST25CSLow();

    SPIPut(SST25_CMD_RD_JEDEC_ID);			// command to read JEDEC ID, clocks in received data
    SPIGet();								// dummy read to clear SPI Rx register

	// **************************************
	SPIPut(0);								// dummy write to clock in 1st ID byte
	*pData = SPIGet();						// get a byte

	if (*pData IS_NOT ID_JEDEC_SST)			// verify JEDEC Manufacturer ID
		bRetVal = FALSE;

	pData++;								// bump output buffer

	// **************************************
	SPIPut(0);								// dummy write to clock in 1st ID byte
	*pData = SPIGet();						// get a byte

	if (*pData IS_NOT ID_SPI_FLASH)			// verify device format
		bRetVal = FALSE;

	pData++;								// bump output buffer

	// **************************************
	SPIPut(0);								// dummy write to clock in 1st ID byte
	*pData = SPIGet();						// get a byte

	if (*pData IS_NOT ID_SST25VF016)		// verify specific device type
		bRetVal = FALSE;


    SST25CSHigh();

	return bRetVal;
}

/************************************************************************
* Function: void SST25WriteByte(BYTE data, DWORD address)                                           
*                                                                       
* Overview: this function writes a byte to the address specified
*                                                                       
* Input: data to be written and address
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void SST25WriteByte(BYTE data, DWORD address)
{
    SST25WriteEnable();
    SST25CSLow();

    SPIPut(SST25_CMD_BYTE_WRITE);			// <sek> was SST25_CMD_WRITE
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    SPIPut(data);
    SPIGet();

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy())
		;
}

/************************************************************************
* Function: BYTE SST25ReadByte(DWORD address)             
*                                                                       
* Overview: this function reads a byte from the address specified         
*                                                                       
* Input: address                                                     
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
BYTE SST25ReadByte(DWORD address)
{
    BYTE    temp;
    SST25CSLow();

    SPIPut(SST25_CMD_READ);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    SPIPut(0);
    temp = SPIGet();

    SST25CSHigh();
    return (temp);
}

/************************************************************************
* Function: void SST25WriteWord(WODR data, DWORD address)                                           
*                                                                       
* Overview: this function writes a 16-bit word to the address specified
*                                                                       
* Input: data to be written and address
*                                                                       
* Output: none                                                         
*                                                                       
************************************************************************/
void SST25WriteWord(WORD data, DWORD address)
{
    SST25WriteByte(((WORD_VAL) data).v[0], address);
    SST25WriteByte(((WORD_VAL) data).v[1], address + 1);
}

/************************************************************************
* Function: WORD SST25ReadWord(DWORD address)             
*                                                                       
* Overview: this function reads a 16-bit word from the address specified         
*                                                                       
* Input: address                                                     
*                                                                       
* Output: data read
*                                                                       
************************************************************************/
WORD SST25ReadWord(DWORD address)
{
    WORD_VAL    temp;

    temp.v[0] = SST25ReadByte(address);
    temp.v[1] = SST25ReadByte(address + 1);

    return (temp.Val);
}

/************************************************************************
* Function: SST25WriteEnable()                                         
*                                                                       
* Overview: this function allows write/erase SST25. Must be called  
* before every write/erase command.                                         
*                                                                       
* Input: none                                                          
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void SST25WriteEnable(void)
{
    SST25CSLow();
    SPIPut(SST25_CMD_WR_EN);				// <sek> was SST25_CMD_WREN
    SPIGet();
    SST25CSHigh();
}

/************************************************************************
* Function: BYTE SST25IsWriteBusy(void)  
*                                                                       
* Overview: this function reads status register and chek BUSY bit for write operation
*                                                                       
* Input: none                                                          
*                                                                       
* Output: non zero if busy
*                                                                       
************************************************************************/
BYTE SST25IsWriteBusy(void)
{
    BYTE    temp;

    SST25CSLow();
    SPIPut(SST25_CMD_RD_STATUS_REG);		// <sek> was SST25_CMD_RDSR
    SPIGet();

    SPIPut(0);
    temp = SPIGet();
    SST25CSHigh();

    return (temp & 0x01);
}

/************************************************************************
* Function: BYTE SST25WriteArray(DWORD address, BYTE* pData, nCount)
*                                                                       
* Overview: this function writes a data array at the address specified
*                                                                       
* Input: flash memory address, pointer to the data array, data number
*                                                                       
* Output: return 1 if the operation was successfull
*                                                                     
************************************************************************/
BYTE SST25WriteArray(DWORD address, BYTE *pData, WORD nCount)
{
    DWORD   addr;
    BYTE    *pD;
    WORD    counter;

    addr = address;
    pD = pData;

    // WRITE
    for(counter = 0; counter < nCount; counter++)
    {
        SST25WriteByte(*pD++, addr++);
    }

    // VERIFY
    for(counter = 0; counter < nCount; counter++)
    {
        if(*pData != SST25ReadByte(address))
            return (0);
        pData++;
        address++;
    }

    return (1);
}

/************************************************************************
* Function: void SST25ReadArray(DWORD address, BYTE* pData, nCount)
*                                                                       
* Overview: this function reads data into buffer specified
*                                                                       
* Input: flash memory address, pointer to the data buffer, data number
*                                                                       
************************************************************************/
void SST25ReadArray(DWORD address, BYTE *pData, WORD nCount)
{
    SST25CSLow();

    SPIPut(SST25_CMD_READ);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    while(nCount--)
    {
        SPIPut(0);
        *pData++ = SPIGet();
    }

    SST25CSHigh();
}

/************************************************************************
* Function: void SST25ChipErase(void)
*                                                                       
* Overview: chip erase
*                                                                       
* Input: none
*                                                                       
************************************************************************/
void SST25ChipErase(void)
{
    SST25WriteEnable();

    SST25CSLow();

    SPIPut(SST25_CMD_CHIP_ERASE);		// <sek> was SST25_CMD_ERASE
    SPIGet();

    SST25CSHigh();

    // Wait for write end
    while(SST25IsWriteBusy())
		;
}

/************************************************************************
* Function: void SST25ResetWriteProtection()
*                                                                       
* Overview: this function reset write protection bits
*                                                                       
* Input: none                                                     
*                                                                       
* Output: none
*                                                                       
************************************************************************/
void SST25ResetWriteProtection(void)
{
    SST25CSLow();

    SPIPut(SST25_CMD_EN_WR_STATUS_REG);		// <sek> was SST25_CMD_EWSR
    SPIGet();

    SST25CSHigh();

    SST25CSLow();

    SPIPut(SST25_CMD_WR_STATUS_REG);		// <sek> was SST25_CMD_WRSR
    SPIGet();

    SPIPut(0);
    SPIGet();

    SST25CSHigh();
}

/************************************************************************
* Function: void SST25SectorErase(DWORD address)                                           
*                                                                       
* Overview: this function erases a 4Kb sector
*                                                                       
* Input: address within sector to be erased
*                                                                       
* Output: none                                 
*                                                                       
************************************************************************/
void SST25SectorErase(DWORD address)
{
    SST25WriteEnable();
    SST25CSLow();

    SPIPut(SST25_CMD_4K_ERASE);			// <sek> was SST25_CMD_SER
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[2]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[1]);
    SPIGet();

    SPIPut(((DWORD_VAL) address).v[0]);
    SPIGet();

    SST25CSHigh();

    // Wait for write end
    DelayMs(100);
    while(SST25IsWriteBusy())
		;
}

