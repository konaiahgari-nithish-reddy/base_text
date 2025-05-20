/***********************************************************************************************\
* Freescale MMA8451,2,3Q Driver
*
* Filename: mma845x.c
*
*
* (c) Copyright 2010, Freescale, Inc.  All rights reserved.
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
\***********************************************************************************************/



#ifndef __32MX360F512L__
	#error	Incorrect Processor Type Selected; code is for PIC32MX360F512L
#endif

//-----------------------------------------------------------------------------
//								#include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>
#include "config.h"					// compile time configuration definitions

//lint -e765						error 765: (Info -- external function could be made static)
//lint -e14							error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>					// Microchip PIC32 peripheral library main header
//lint +e14

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"

#include "HardwareProfile.h"

#include "I2CBus.h"
#include "mma845x.h"              // MMA845xQ macros

#include "TimeDelay.h"

#ifdef USE_MMA8452Q_INCLINOMETER

/***********************************************************************************************\
* Private macros
\***********************************************************************************************/

/***********************************************************************************************\
* Private type definitions
\***********************************************************************************************/

/***********************************************************************************************\
* Private prototypes
\***********************************************************************************************/

/***********************************************************************************************\
* Private memory declarations
\***********************************************************************************************/

/***********************************************************************************************\
* Public memory declarations
\***********************************************************************************************/

//extern byte SlaveAddressIIC;


/***********************************************************************************************\
* Public functions
\***********************************************************************************************/

/*********************************************************\
* Put MMA845xQ into Active Mode
\*********************************************************/
BOOL MMA845x_Active ()
{
	BOOL bRetVal = TRUE;
	BYTE bTemp;

	//IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) | ACTIVE_MASK));

	bRetVal = ReadInclinometerRegister(CTRL_REG1, &bTemp);
	if (bRetVal IS TRUE)
	{
		bRetVal = WriteInclinometerRegister(CTRL_REG1, bTemp | ACTIVE_MASK);
	}

	return bRetVal;
}

/*********************************************************\
* Put MMA845xQ into Standby Mode
\*********************************************************/
BOOL MMA845x_Standby (void)
{
//	byte n;
	BYTE bTemp;
	BOOL bRetVal = TRUE;

	/*
	**  Read current value of System Control 1 Register.
	**  Put sensor into Standby Mode.
	**  Return with previous value of System Control 1 Register.
	*/
//	n = IIC_RegRead(SlaveAddressIIC, CTRL_REG1);
//	IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, n & ~ACTIVE_MASK);

	bRetVal = ReadInclinometerRegister(CTRL_REG1, &bTemp);
	if (bRetVal IS TRUE)
	{
		bRetVal = WriteInclinometerRegister(CTRL_REG1, bTemp & ~ACTIVE_MASK);
	}

	return bRetVal;
}

/*********************************************************\
* Initialize MMA845xQ
\*********************************************************/
BOOL MMA845x_Init (void)
{
	BYTE bTemp;
	BOOL bRetVal = TRUE;

	// Note: Except for STANDBY mode selection, the device must be in STANDBY mode to change any of the fields within CTRL_REG1 (0X2A) See below
	bRetVal = MMA845x_Standby();
	if (bRetVal IS FALSE)
		return bRetVal;

//	IIC_RegRead(SlaveAddressIIC, WHO_AM_I_REG);
	bRetVal = ReadInclinometerRegister(WHO_AM_I_REG, &bTemp);

	if ((bTemp IS MMA8452Q_ID) AND (bRetVal IS TRUE))
		bRetVal = TRUE;
	else
		return (FALSE);

	/*
	**  Configure sensor for:
	**    - Sleep Mode Poll Rate of 50Hz (20ms)
	**    - System Output Data Rate of 200Hz (5ms)
	**    - Full Scale of +/-2g
	*/

//	IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, ASLP_RATE_20MS+DATA_RATE_5MS);
//	IIC_RegWrite(SlaveAddressIIC, XYZ_DATA_CFG_REG, FULL_SCALE_2G);

	bRetVal = WriteInclinometerRegister(CTRL_REG1, ASLP_RATE_20MS + DATA_RATE_5MS);
	if (bRetVal IS FALSE)
		return bRetVal;

	bRetVal = WriteInclinometerRegister(XYZ_DATA_CFG_REG, FULL_SCALE_2G);

	return bRetVal;

}


#ifdef NOTDEF
void initMMA8452(byte fsr, byte dataRate)
{
  MMA8452Standby();  // Must be in standby to change registers

  /* Set up the full scale range to 2, 4, or 8g. */
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(0x0E, fsr >> 2);
  else
    writeRegister(0x0E, 0);
  /* Setup the 3 data rate bits, from 0 to 7 */
  writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));

  /* Set up interrupt 1 and 2 */
  writeRegister(0x2C, 0x02);  // Active high, push-pull
  writeRegister(0x2D, 0x19);  // DRDY int enabled, P/L enabled
  writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L on INT2

  MMA8452Active();  // Set to active to start reading
}
#endif


/************************************************************************
* Function: BOOL ReadInclinometerRegister(UINT8 cRegAddress, UINT8 *ptrData)
*
* Overview: this function reads a single RTCC Register into buffer specified
*
* Input: register offset from 0, pointer to the data buffer
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/

BOOL ReadInclinometerRegister(UINT8 cRegAddress, UINT8 *ptrData)
{
	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(MMA8452Q_I2C_BUS, TRUE);

	if (read_i2c_device(MMA8452Q_I2C_ADDR, cRegAddress, ptrData, 1)	IS FALSE)
	{
		bRetVal = FALSE;
	}

    // Disable the I2C bus
    I2CEnable(MMA8452Q_I2C_BUS, FALSE);

	return bRetVal;
}


/************************************************************************
* Function: BOOL WriteInclinometerRegister(UINT8 cRegAddress, UINT8 bData)
*
* Overview: this function writes a single RTCC Register
*
* Input: register offset from 0, data
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/
BOOL WriteInclinometerRegister(UINT8 cRegAddress, UINT8 bData)
{

	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(MMA8452Q_I2C_BUS, TRUE);

	if (write_i2c_device(MMA8452Q_I2C_ADDR, cRegAddress, &bData, 1)	IS FALSE)
	{
		bRetVal = FALSE;
	}

    // Disable the I2C bus
    I2CEnable(MMA8452Q_I2C_BUS, FALSE);

	return bRetVal;
}

/************************************************************************
* Function: BOOL ReadInclinometerArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen)
*
* Overview: this function reads RTCC RAM data into buffer specified
*
* Input: offset from base RTCC RAM address, pointer to the data buffer, size of data
*
* Output: return TRUE if the operation was successfull
*
************************************************************************/


BOOL ReadInclinometerArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen)
{
	BOOL bRetVal = TRUE;

    // Enable the I2C bus
    I2CEnable(MMA8452Q_I2C_BUS, TRUE);

	#ifdef USE_INCLINOMETER_BLOCK_READ
		// read all bytes in a single call to read_i2c_device()
		if (read_i2c_device(MMA8452Q_I2C_ADDR, cDataAddress, ptrData, cDataLen) IS FALSE)
		{
			bRetVal = FALSE;
		}
	#else
		// read one byte per call to read_i2c_device(); implemented because of problems with read_i2c_device() (?)
		//		**may** be related to problems with i2C STOP
		while(cDataLen--)
		{
			if (read_i2c_device(MMA8452Q_I2C_ADDR, cDataAddress, ptrData, 1)	IS FALSE)
			{
				bRetVal = FALSE;
				break;
			}

			cDataAddress++;
			ptrData++;					// bump data storage pointer
		}
	#endif


    // Disable the I2C bus
    I2CEnable(MMA8452Q_I2C_BUS, FALSE);

	return bRetVal;
}

/***********************************************************************************************\
* Private functions
\***********************************************************************************************/

#endif	//  USE_MMA8452Q_INCLINOMETER
// end of MMA845x.c

