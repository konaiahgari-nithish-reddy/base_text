/*
 * PeripheralCallbacks.c
 *
 *  Created on: 15-Jul-2023
 *      Author: Ravi's PC
 */
#include <stdio.h>
#include "main.h"
#include "PeripheralCallbacks.h"


int SpiTransmit(SPI_HandleTypeDef *handle, const uint8_t* pData, uint16_t Size)
{
	uint16_t ret;

	ret = HAL_SPI_Transmit(handle, pData, Size, 2);
	if(ret != HAL_OK)
	{
		printf("SPI Transmit Failed\n");
	}
	return ret;
}

int SpiReceive(SPI_HandleTypeDef *handle, uint8_t* pData, uint16_t Size)
{
	uint16_t ret;

	ret = HAL_SPI_Receive(handle, pData, Size, 2);
	if(ret != HAL_OK)
	{
		printf("SPI Receive Failed\n");
	}
	return ret;
}

uint8_t* SpiTransmitReceive(SPI_HandleTypeDef *handle, uint8_t* pData, uint16_t Size)
{
	uint16_t ret;
	uint8_t* rxData;

	ret = HAL_SPI_TransmitReceive(handle, pData, rxData, Size, 2);
	if(ret != HAL_OK)
	{
		printf("SPI Receive Failed\n");
	}
	return rxData;
}

int I2CTransmit(I2C_HandleTypeDef *handle, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	uint16_t ret;

	ret = HAL_I2C_Master_Transmit(handle, DevAddress, pData, Size, 2);
	if(ret != HAL_OK)
	{
		printf("I2C Transmit Failed\n");
	}
	return ret;
}

int I2CReceive(I2C_HandleTypeDef *handle, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	uint16_t ret;

	ret = HAL_I2C_Master_Receive(handle, DevAddress, pData, Size, 2);
	if(ret != HAL_OK)
	{
		printf("I2C Receive Failed\n");
	}
	return ret;
}

int UartTransmit(UART_HandleTypeDef *handle, uint8_t *pData, uint16_t Size)
{
	uint16_t ret;

	ret = HAL_UART_Transmit(handle, pData, Size, 200);
	if(ret != HAL_OK)
	{
		printf("Uart Transmit Failed\n");
	}
	return ret;
}

int UartReceive(UART_HandleTypeDef *handle, uint8_t *pData, uint16_t Size)
{
	uint16_t ret;

	ret = HAL_UART_Receive(handle, pData, Size, 1);
	if(ret != HAL_OK)
	{
		printf("Uart Receive Failed\n");
	}
	return ret;
}
