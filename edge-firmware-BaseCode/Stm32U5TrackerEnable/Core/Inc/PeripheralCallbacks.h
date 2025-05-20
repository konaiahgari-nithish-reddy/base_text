/*
 * PeripheralCallbacks.h
 *
 *  Created on: 15-Jul-2023
 *      Author: Ravi's PC
 */

#ifndef INC_PERIPHERALCALLBACKS_H_
#define INC_PERIPHERALCALLBACKS_H_

#include "stm32u5xx_hal.h"
#define TIMEOUT	120

int SpiTransmit(SPI_HandleTypeDef *handle, const uint8_t* pData, uint16_t Size);
int SpiReceive(SPI_HandleTypeDef *handle, uint8_t* pData, uint16_t Size);
int I2CTransmit(I2C_HandleTypeDef *handle, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
int I2CReceive(I2C_HandleTypeDef *handle, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
int UartTransmit(UART_HandleTypeDef *handle, uint8_t *pData, uint16_t Size);
int UartReceive(UART_HandleTypeDef *handle, uint8_t *pData, uint16_t Size);

#endif /* INC_PERIPHERALCALLBACKS_H_ */
