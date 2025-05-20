#ifndef __SST25V_FLASH_CONFIG_H
#define __SST25V_FLASH_CONFIG_H

#include "stm32u5xx_hal.h"

#define _SST25_USE_FREERTOS          0
#define _SST25_DEBUG                 0

extern SPI_HandleTypeDef hspi2;
#define SST25_hspi				hspi2

#define	FLASH_SS_Pin			GPIO_PIN_0
#define	FLASH_SS_GPIO_Port		GPIOD

#define FLASH_MISO_GPIO_Port	GPIOD
#define FLASH_MISO_Pin			GPIO_PIN_3
#endif
