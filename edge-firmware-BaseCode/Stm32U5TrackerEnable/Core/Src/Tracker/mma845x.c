#include <stdio.h>
#include <stdint.h>
#include"mma845x.h"
#include "main.h"
#include "Peripheralcallbacks.h"
#include "stm32u5xx_hal.h"
#include "LIS2HH12_Accelerometer.h"

extern I2C_HandleTypeDef hi2c1;
extern stmdev_ctx_t dev_ctx;;
/*********************************************************\
* Initialize MMA845xQ
\*********************************************************/
extern BOOL gIsMma845Enabled;
/*********************************************************\
* Put MMA845xQ into Active Mode
\*********************************************************/

BOOL read_i2c_device(UINT8 slaveaddr,UINT8 address,UINT8 *Data,UINT8 cDataLen)
{
//	UINT16 Address;

//	Address = slaveaddr;
//	Address = (Address << 8) & address;
//	printf("Address:%x\n", Address);
//	if (I2CReceive(&hi2c1, Address, Data, cDataLen) == HAL_OK)

	if (HAL_I2C_Mem_Read(&hi2c1, slaveaddr, address, 1, Data, cDataLen, 100) == HAL_OK) {
		return TRUE;
	}

	return FALSE;
}

BOOL write_i2c_device(UINT8 slaveaddr,UINT8 address,UINT8 *Data,UINT8 cDataLen)
{
//	UINT16 Address;
//
//	Address = slaveaddr;
//	Address = (Address << 8) & address;
//	printf("Address:%x\n", Address);

	if (HAL_I2C_Mem_Write(&hi2c1, slaveaddr, address, 1, Data, cDataLen, 100) == HAL_OK)
		return TRUE;
//	if (I2CTransmit(&hi2c1, Address, Data, cDataLen) == HAL_OK)
//		return TRUE;
	return FALSE;
}

BOOL MMA845x_Active ()
{
	UINT8 bTemp;
	BOOL bRetVal = TRUE;
	

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
	UINT8 bTemp;
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

	if ((bTemp IS MMA8452Q_ID) && (bRetVal IS TRUE))
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

	if (gIsMma845Enabled)
	{
		if (read_i2c_device(MMA8452Q_I2C_ADDR, cRegAddress, ptrData, 1)	IS FALSE)
		{
			bRetVal = FALSE;
		}
	}
	else
	{
		int32_t ret;

		ret = lis2hh12_read_reg(&dev_ctx, cRegAddress, ptrData, 1);

		if (ret != 0)
			bRetVal = FALSE;
	}

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

	if (gIsMma845Enabled)
	{
		if (write_i2c_device(MMA8452Q_I2C_ADDR, cRegAddress, &bData, 1)	IS FALSE)
		{
			bRetVal = FALSE;
		}
	}
	else
	{
		int32_t ret;

		ret = lis2hh12_write_reg(&dev_ctx, cRegAddress, &bData, 1);

		if (ret != 0)
			bRetVal = FALSE;
	}

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
		if (gIsMma845Enabled)
		{
			if (read_i2c_device(MMA8452Q_I2C_ADDR, cDataAddress, ptrData, 1)	IS FALSE)
			{
				bRetVal = FALSE;
				break;
			}
		}
		else
		{
			int32_t ret;

//			ret = lis2hh12_read_reg(&dev_ctx, cDataAddress, ptrData, 1);
			memset(ptrData, 0x00, 3 * sizeof(int16_t));
			ret = lis2hh12_acceleration_raw_get(&dev_ctx, (int16_t*)ptrData);

			if (ret != 0)
				bRetVal = FALSE;
			else {
				bRetVal = TRUE;
				return bRetVal;
			}
		}
		cDataAddress++;
		ptrData++;					// bump data storage pointer
	}
	#endif

	return bRetVal;
}
