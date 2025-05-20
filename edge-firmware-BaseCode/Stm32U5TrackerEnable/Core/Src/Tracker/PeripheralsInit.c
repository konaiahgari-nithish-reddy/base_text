/*
 * PeripheralsInit.c
 *
 *  Created on: 01-Sep-2023
 *      Author: Ravi's PC
 */

#include "main.h"

void SPI_Flash_Init(void)
{
	MX_SPI2_Init();
}

void SPI_ESP32_Init(void)
{
	MX_SPI1_Init();
}

void SPI_Driver_Init(void)
{
	MX_SPI3_Init();
}

void I2C_Accelerometer_Init(void)
{
	MX_I2C1_Init();
}

void ESP32_UART_Init(void)
{
	MX_LPUART1_UART_Init();
}

void ADC_VI_CI_Init(void)
{
	MX_ADC1_Init();
}
/*
void Serial_Uart_Init(void)
{
	MX_USART3_UART_Init();
}
*/
void USB_Serial_Init(void)
{
	MX_USB_OTG_FS_PCD_Init();
}
