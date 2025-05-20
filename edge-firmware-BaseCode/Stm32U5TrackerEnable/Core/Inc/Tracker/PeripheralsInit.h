/*
 * PeripheralsInit.h
 *
 *  Created on: 01-Sep-2023
 *      Author: Ravi's PC
 */

#ifndef INC_TRACKER_PERIPHERALSINIT_H_
#define INC_TRACKER_PERIPHERALSINIT_H_

void SPI_Flash_Init(void);
void SPI_ESP32_Init(void);
void SPI_Driver_Init(void);
void I2C_Accelerometer_Init(void);
void ESP32_UART_Init(void);
void ADC_VI_CI_Init(void);
void Serial_Uart_Init(void);
void USB_Serial_Init(void);

#endif /* INC_TRACKER_PERIPHERALSINIT_H_ */
