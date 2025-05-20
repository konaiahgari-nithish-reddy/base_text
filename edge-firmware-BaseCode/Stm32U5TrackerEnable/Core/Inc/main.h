/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PRESCALER_VALUE     (uint32_t)(((SystemCoreClock) / 1000000) - 1)
#define  PERIOD_VALUE       (uint32_t)(41 - 1)
/* Uncomment to enable the adequate Clock Source */
#define RTC_CLOCK_SOURCE_LSI
/*#define RTC_CLOCK_SOURCE_LSE*/

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xF9
#endif
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_I2C1_Init(void);
void MX_LPUART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_MEMORYMAP_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_TIM2_Init(void);
void MX_USB_OTG_FS_USB_Init(void);
void MX_ICACHE_Init(void);
void MX_TIM4_Init(void);
void MX_TIM6_Init(uint16_t Period);
void MX_TIM7_Init(uint16_t Period);
void MX_RTC_Init(void);
void InitializeParameretes(void);
void InitializeRTC(void);
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Wifi_Uart_Tx_Pin GPIO_PIN_0
#define Wifi_Uart_Tx_GPIO_Port GPIOC
#define Wifi_Uart_RX_Pin GPIO_PIN_1
#define Wifi_Uart_RX_GPIO_Port GPIOC
#define VIn_Sense_Pin GPIO_PIN_3
#define VIn_Sense_GPIO_Port GPIOC
#define DRIVE_PH_IN2_TIM_CH1_Pin GPIO_PIN_0
#define DRIVE_PH_IN2_TIM_CH1_GPIO_Port GPIOA
#define DRIVE_EN_IN1_TIM_CH2_Pin GPIO_PIN_1
#define DRIVE_EN_IN1_TIM_CH2_GPIO_Port GPIOA
#define DRIVE_IPROPI_ADC_Pin GPIO_PIN_2
#define DRIVE_IPROPI_ADC_GPIO_Port GPIOA
#define DRIVE_nFAULT_Pin GPIO_PIN_3
#define DRIVE_nFAULT_GPIO_Port GPIOA
#define DRIVE_DIAG_Pin GPIO_PIN_4
#define DRIVE_DIAG_GPIO_Port GPIOA
#define DRIVE_DRVOFF_Pin GPIO_PIN_5
#define DRIVE_DRVOFF_GPIO_Port GPIOA
#define DRIVE_nSLEEP_Pin GPIO_PIN_6
#define DRIVE_nSLEEP_GPIO_Port GPIOA
#define Wind_Input_Pin GPIO_PIN_7
#define Wind_Input_GPIO_Port GPIOA
#define Wifi_Enable_Pin GPIO_PIN_11
#define Wifi_Enable_GPIO_Port GPIOE
#define Wifi_SPI1_CS_Pin GPIO_PIN_12
#define Wifi_SPI1_CS_GPIO_Port GPIOE
#define Wifi_SPI1_SCK_Pin GPIO_PIN_13
#define Wifi_SPI1_SCK_GPIO_Port GPIOE
#define Wifi_SPI1_MISO_Pin GPIO_PIN_14
#define Wifi_SPI1_MISO_GPIO_Port GPIOE
#define Wifi_SPI1_MOSI_Pin GPIO_PIN_15
#define Wifi_SPI1_MOSI_GPIO_Port GPIOE
#define SERIAL_TX_Pin GPIO_PIN_10
#define SERIAL_TX_GPIO_Port GPIOB
#define SERIAL_RX_Pin GPIO_PIN_11
#define SERIAL_RX_GPIO_Port GPIOB
#define ACCEL_INT1_Pin GPIO_PIN_14
#define ACCEL_INT1_GPIO_Port GPIOB
#define ACCEL_INT2_Pin GPIO_PIN_15
#define ACCEL_INT2_GPIO_Port GPIOB
#define DRIVE_SR_Pin GPIO_PIN_10
#define DRIVE_SR_GPIO_Port GPIOC
#define DRIVE_ITRIP_Pin GPIO_PIN_11
#define DRIVE_ITRIP_GPIO_Port GPIOC
#define DRIVE_MODE_Pin GPIO_PIN_12
#define DRIVE_MODE_GPIO_Port GPIOC
#define Flash_SPI2_CS_Pin GPIO_PIN_0
#define Flash_SPI2_CS_GPIO_Port GPIOD
#define Flash_SPI2_Sck_Pin GPIO_PIN_1
#define Flash_SPI2_Sck_GPIO_Port GPIOD
#define Flash_SPI2_MISO_Pin GPIO_PIN_3
#define Flash_SPI2_MISO_GPIO_Port GPIOD
#define Flash_SPI2_MOSI_Pin GPIO_PIN_4
#define Flash_SPI2_MOSI_GPIO_Port GPIOD
#define Flash_Write_Prot_Pin GPIO_PIN_5
#define Flash_Write_Prot_GPIO_Port GPIOD
#define Flash_Hold_Pin GPIO_PIN_6
#define Flash_Hold_GPIO_Port GPIOD
#define Accel_I2C_SCL_Pin GPIO_PIN_6
#define Accel_I2C_SCL_GPIO_Port GPIOB
#define Accel_I2C_SDA_Pin GPIO_PIN_7
#define Accel_I2C_SDA_GPIO_Port GPIOB
#define PB_Manual_Pin GPIO_PIN_8
#define PB_Manual_GPIO_Port GPIOB
#define PB_Auto_Pin GPIO_PIN_9
#define PB_Auto_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void refresh(void);
void refresh_delay(uint8_t seconds);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
