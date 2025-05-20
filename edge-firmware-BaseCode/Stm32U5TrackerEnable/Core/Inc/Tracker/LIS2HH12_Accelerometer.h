/*
 * LIS2HH12_Accelerometer.h
 *
 *  Created on: 07-Oct-2023
 *      Author: Ravi's PC
 */

#ifndef INC_TRACKER_LIS2HH12_ACCELEROMETER_H_
#define INC_TRACKER_LIS2HH12_ACCELEROMETER_H_

#include "lis2hh12_reg.h"
#include "GenericTypedefs.h"

#define    BOOT_TIME   20 //ms

void LIS2HH12TR_ReadData(uint8_t regAddr, uint8_t* data, uint16_t size);
void LIS2HH12TR_WriteData(uint8_t regAddr, uint8_t* data, uint16_t size);
void LIS2HH12_ReadAccData(int16_t* accData);
void LIS2HH12_Init(void);
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t ms);
BOOL Read_WhoAmI_Register();
BOOL Enable_XYZ_Axis(void);

#endif /* INC_TRACKER_LIS2HH12_ACCELEROMETER_H_ */
