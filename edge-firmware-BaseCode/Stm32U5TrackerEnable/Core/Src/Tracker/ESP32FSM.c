/*
 * ESP32FSM.c
 *
 *  Created on: 29-Oct-2023
 *      Author: Ravi's PC
 */
#include "stm32u5xx_hal.h"
#include "PeripheralCallbacks.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef hlpuart1;

void WIFI_Uart_Send(unsigned char *buf)
{
	UartTransmit(&hlpuart1, buf, strlen(buf));
}

int WIFI_Uart_Recv(unsigned char *buf,int size)
{
	int ret = UartReceive(&hlpuart1, buf, size);
	return ret;
}


