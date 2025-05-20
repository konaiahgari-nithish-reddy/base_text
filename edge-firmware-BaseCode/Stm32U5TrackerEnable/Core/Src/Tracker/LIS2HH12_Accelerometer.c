/*
 * LIS2HH12_Accelerometer.c
 *
 *  Created on: 07-Oct-2023
 *      Author: Ravi
 */
#include "stm32u5xx_hal.h"
#include "PeripheralCallbacks.h"
#include "LIS2HH12_Accelerometer.h"
#include "main.h"
#include <string.h>

#define LIS2HH12_I2C_ADDRESS_WRITE 0x3D // I2c Address for Read 0011 1011 (0x3B) for write 0011 1010(0x3A)
#define LIS2HH12_I2C_ADDRESS_READ  0x3C
#define ACCELEROMETER_DATA_SIZE 6

static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;

stmdev_ctx_t dev_ctx;

extern I2C_HandleTypeDef hi2c1;

BOOL Enable_XYZ_Axis(void)
{
	// Configure CTRL1: X, Y, Z enabled, ODR = 100 Hz, BDU enabled
	uint8_t ctrl1Reg = 0x20;
	uint8_t ctrl1Data = 0x3F;

	if (HAL_I2C_Mem_Write(&hi2c1, LIS2HH12_I2C_ADD_L, ctrl1Reg, 1, &ctrl1Data, 1, 1000) != HAL_OK)
	{
		return FALSE;
	}

	return TRUE;
}

void LIS2HH12_Init(void)
{
  /*  Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;
	lis2hh12_xl_axis_t xyz;
	xyz.xen = 1;
	xyz.yen = 1;
	xyz.zen = 1;
	/* Initialize platform specific hardware */
	/* Wait sensor boot time */
	platform_delay(20);

	/* Check device ID */
	Read_WhoAmI_Register();

	/* Restore default configuration */
	lis2hh12_dev_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		lis2hh12_dev_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lis2hh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set full scale */
	lis2hh12_xl_full_scale_set(&dev_ctx, LIS2HH12_8g);
	/* Enable XYZ to read data */
	lis2hh12_xl_axis_set(&dev_ctx, xyz);
	/* Configure filtering chain */
	/* Accelerometer data output- filter path / bandwidth */
	lis2hh12_xl_filter_aalias_bandwidth_set(&dev_ctx, LIS2HH12_AUTO);
	lis2hh12_xl_filter_out_path_set(&dev_ctx, LIS2HH12_FILT_LP);
	lis2hh12_xl_filter_low_bandwidth_set(&dev_ctx,
									   LIS2HH12_LP_ODR_DIV_400);
	/* Accelerometer interrupt - filter path / bandwidth */
	lis2hh12_xl_filter_int_path_set(&dev_ctx, LIS2HH12_HP_DISABLE);
	/* Set Output Data Rate */
	lis2hh12_xl_data_rate_set(&dev_ctx, LIS2HH12_XL_ODR_100Hz);
}

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, LIS2HH12_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LIS2HH12_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}



void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

BOOL Read_WhoAmI_Register()
{
	  lis2hh12_dev_id_get(&dev_ctx, &whoamI);

	  if (whoamI != LIS2HH12_ID)
	    return FALSE;

	  return TRUE;
}


void LIS2HH12_ReadAccData(int16_t* accData) {
  // Poll STATUS register for new data
  uint8_t statusReg = 0x27;
  do {
    if (HAL_I2C_Master_Receive(&hi2c1, LIS2HH12_I2C_ADDRESS_READ, &statusReg, 1, 100) != HAL_OK) {
      Error_Handler();
    }
  } while ((statusReg & 0x08) == 0); // Wait for ZYXDA bit to be set

  // Read acceleration data
  uint8_t data[ACCELEROMETER_DATA_SIZE];
  uint8_t Out_X_L_reg = 0x28;
  if (HAL_I2C_Mem_Read(&hi2c1, LIS2HH12_I2C_ADDRESS_READ, Out_X_L_reg, 1, data, ACCELEROMETER_DATA_SIZE, 100) != HAL_OK) {
    Error_Handler();
  }

  // Convert raw data to 16-bit values
  accData[0] = (int16_t)((data[1] << 8) | data[0]);
  accData[1] = (int16_t)((data[3] << 8) | data[2]);
  accData[2] = (int16_t)((data[5] << 8) | data[4]);
}
