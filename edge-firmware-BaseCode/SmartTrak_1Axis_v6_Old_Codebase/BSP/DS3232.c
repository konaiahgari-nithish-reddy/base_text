// *************************************************************************************************
//										D S 3 2 3 2 . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	DS3232 RTCC Functions
//
// *************************************************************************************************

// These functions are NOT register level. They call into the PIC32 Peripheral library to
// handle most register access, especially initialization

// NOTE: Write Array functions now use a single call to write_i2C_device() to write multiple bytes.
//		 Read Array functions still use one call to read_i2c_device() per byte, because of previously observed problems
//				this MAY be changed, pending additional testing.

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

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"
#include "StrConversions.h"

#include "I2CBus.h"
#include "DS3232.h"

// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
// ****************************************************************************

// return RTCC Date and Time in a structure of BYTES and INTs, converted from BCD
// NOTE: implementee as one byte per call to read_i2c_device() because of varying conversion code.

BOOL ReadRTCCDateTime(PTR_RTCC_DATE_TIME ptrDateTime)
{
	UINT8 cI2CData;
	BOOL bRetval = TRUE;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	if (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_SECONDS, &cI2CData, 1)	IS TRUE)
	{
		ptrDateTime->cSeconds = BCDtoBYTE(cI2CData & DS3232_SECONDS_MASK);		// read OK, convert BCD to byte
	}
	else
	{
		bRetval = FALSE;														// did NOT read OK
	}


	if ((bRetval IS TRUE) AND (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_MINUTES, &cI2CData, 1)	IS TRUE))
	{
		ptrDateTime->cMinutes = BCDtoBYTE(cI2CData & DS3232_MINUTES_MASK);		// read OK, convert BCD to Byte
	}
	else
	{
		bRetval = FALSE;														// did NOT read OK
	}


	if ((bRetval IS TRUE) AND (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_HOURS, &cI2CData, 1)	IS TRUE))
	{
		if ((cI2CData & DS3232_12_NOT24_HOURS_MASK) IS 0)						// 24 hours?
		{
			ptrDateTime->cHours = BCDtoBYTE(cI2CData & DS3232_24_HOURS_MASK);	// read OK
			ptrDateTime->b12_24 = 0;
		}
		else			// 12 hours
		{
			ptrDateTime->cHours = BCDtoBYTE(cI2CData & DS3232_12_HOURS_MASK);	// read OK
			ptrDateTime->b12_24 = 1;
			if ((cI2CData & DS3232_AM_NOTPM_MASK) IS 0)
				ptrDateTime->bAM_PM = 0;
			else
				ptrDateTime->bAM_PM = 1;
		}
	}
	else
	{
		bRetval = FALSE;														// did NOT read OK
	}

	if ((bRetval IS TRUE) AND (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_DAY, &cI2CData, 1)	IS TRUE))
	{
		//ptrDateTime->cDay = BCDtoBYTE(cI2CData & DS3232_DAY_MASK);				// read OK
		ptrDateTime->cDay = cI2CData & DS3232_DAY_MASK;							// read OK, no need for BCD conversion of single digit
	}
	else
	{
		bRetval = FALSE;														// did NOT read OK
	}

	if ((bRetval IS TRUE) AND (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_DATE, &cI2CData, 1)	IS TRUE))
	{
		ptrDateTime->cDate = BCDtoBYTE(cI2CData & DS3232_DATE_MASK);			// read OK
	}
	else
	{
		bRetval = FALSE;														// did NOT read OK
	}

	if ((bRetval IS TRUE) AND (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_MONTH, &cI2CData, 1)	IS TRUE))
	{
		ptrDateTime->cMonth = BCDtoBYTE(cI2CData & DS3232_MONTH_MASK);			// read OK
	}
	else
	{
		bRetval = FALSE;														// did NOT read OK
	}

	if ((bRetval IS TRUE) AND (read_i2c_device(DS3232_I2C_ADDR, DS3232_REG_YEAR, &cI2CData, 1)	IS TRUE))
	{
		ptrDateTime->nYear = (UINT16)BCDtoBYTE(cI2CData & DS3232_YEAR_MASK) + 2000;		// read OK, add 2000 to get complete year number
		if ((cI2CData & DS3232_CENTURY_MASK) IS 0)
			ptrDateTime->bCentury = 0;
		else
			ptrDateTime->bCentury = 1;
	}
	else
	{
		bRetval = FALSE;									// did NOT read OK
	}

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return bRetval;
}


/************************************************************************
* Function: BOOL WriteRTCCRAMArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen)
*
* Overview: this function writes a data array at the RTCC RAM address specified
*
* Input: offset from base RTCC RAM address, pointer to the data buffer, size of data
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/
BOOL WriteRTCCRAMArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen)
{

	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

#ifdef USE_I2C_RTCC_BLOCK_WRITE
	// write all bytes in a single call to write_i2c_device()
	if (write_i2c_device(DS3232_I2C_ADDR, DS3232_SRAM_START + cDataAddress, ptrData, cDataLen)	IS FALSE)
	{
		bRetVal = FALSE;
	}
#else
	// write one byte per call to write_i2c_device(); implemented because of problems with read_i2c_device() (?)
    while(cDataLen--)
    {
		if (write_i2c_device(DS3232_I2C_ADDR, DS3232_SRAM_START + cDataAddress, ptrData, 1)	IS FALSE)
		{
			bRetVal = FALSE;
			break;
		}

		cDataAddress++;
        ptrData++;					// bump data storage pointer
    }
#endif

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return bRetVal;
}

/************************************************************************
* Function: BOOL ClearRTCCRAMArray(UINT8 cDataAddress, UINT8 cDataLen)
*
* Overview: this function clears the RTCC RAM starting at the RTCC RAM address specified
*
* Input: offset from base RTCC RAM address, size of data to clear
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/
BOOL ClearRTCCRAMArray(UINT8 cDataAddress, UINT8 cDataLen)
{

	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

#ifdef USE_I2C_RTCC_BLOCK_WRITE
	// write all bytes in a single call to write_i2c_device()
	if (write_i2c_device(DS3232_I2C_ADDR, DS3232_SRAM_START + cDataAddress, 0x00, cDataLen)	IS FALSE)
	{
		bRetVal = FALSE;
	}
#else
	// write one byte per call to write_i2c_device(); implemented because of problems with read_i2c_device() (?)
    while(cDataLen--)
    {
		if (write_i2c_device(DS3232_I2C_ADDR, DS3232_SRAM_START + cDataAddress, 0x00, 1)	IS FALSE)
		{
			bRetVal = FALSE;
			break;
		}

		cDataAddress++;				// bump address offset
    }
#endif

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return bRetVal;
}



/************************************************************************
* Function: BOOL ReadRTCCRAMArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen)
*
* Overview: this function reads RTCC RAM data into buffer specified
*
* Input: offset from base RTCC RAM address, pointer to the data buffer, size of data
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/

BOOL ReadRTCCRAMArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen)
{
	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	// read one byte per call to read_i2c_device(); implemented because of problems with read_i2c_device() (?)
	//		**may** be related to problems with i2C STOP
    while(cDataLen--)
    {
		if (read_i2c_device(DS3232_I2C_ADDR, DS3232_SRAM_START + cDataAddress, ptrData, 1)	IS FALSE)
		{
			bRetVal = FALSE;
			break;
		}

		cDataAddress++;
        ptrData++;					// bump data storage pointer
    }


    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return bRetVal;
}

/************************************************************************
* Function: BOOL ReadRTCCRegister(UINT8 cRegAddress, UINT8 *ptrData)
*
* Overview: this function reads a single RTCC Register into buffer specified
*
* Input: register offset from 0, pointer to the data buffer
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/

BOOL ReadRTCCRegister(UINT8 cRegAddress, UINT8 *ptrData)
{
	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	if (read_i2c_device(DS3232_I2C_ADDR, cRegAddress, ptrData, 1)	IS FALSE)
	{
		bRetVal = FALSE;
	}

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return bRetVal;
}


/************************************************************************
* Function: BOOL WriteRTCCRegister(UINT8 cRegAddress, UINT8 bData)
*
* Overview: this function writes a single RTCC Register
*
* Input: register offset from 0, data
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/
BOOL WriteRTCCRegister(UINT8 cRegAddress, UINT8 bData)
{

	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	if (write_i2c_device(DS3232_I2C_ADDR, cRegAddress, &bData, 1)	IS FALSE)
	{
		bRetVal = FALSE;
	}

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return bRetVal;
}


// end of DS3232.c

