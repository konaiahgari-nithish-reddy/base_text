/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "main.h"
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
USBD_CDC_LineCodingTypeDef LineCoding = {
  115200,                       /* baud rate */
  0x00,                         /* stop bits-1 */
  0x00,                         /* parity - none */
  0x08                          /* nb. of bits 8 */
};

uint8_t UserRxBuffer[APP_RX_DATA_SIZE]; /* Received Data over USB are stored in
                                         * this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE]; /* Received Data over UART (CDC
                                         * interface) are stored in this buffer
                                         */
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t buf_usb[8];
uint8_t dummy[20] = { 0 };
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */

static int8_t Init(void);
static int8_t DeInit(void);
static int8_t Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t Receive(uint8_t *pbuf, uint32_t *Len);
static int8_t TransmitCplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
static void ComPort_Config(void);


USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  Init,
  DeInit,
  Control,
  Receive,
  TransmitCplt
};

USBD_CDC_LineCodingTypeDef linecoding =
{
  115200, /* baud rate*/
  0x00,   /* stop bits-1*/
  0x00,   /* parity - none*/
  0x08    /* nb. of bits 8*/
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Init(void)
{
  /*
     Add your initialization code here
  */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBuffer);
  return (USBD_OK);
}

/**
  * @brief  DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DeInit(void)
{
  /*
     Add your deinitialization code here
  */

  return (USBD_OK);
}


/**
  * @brief  Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  UNUSED(length);

  switch (cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      /* Add your code here */
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
      /* Add your code here */
      break;

    case CDC_SET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_GET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_CLEAR_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_SET_LINE_CODING:
//    	memcpy(dummy, (uint8_t *)pbuf, length);
//    	linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | W (pbuf[2] << 16) | (pbuf[3] << 24));
//    	linecoding.format     = pbuf[4];
//    	linecoding.paritytype = pbuf[5];
//    	linecoding.datatype   = pbuf[6];
    	buf_usb[0] = pbuf[0];
    	buf_usb[1] = pbuf[1];
    	buf_usb[2] = pbuf[2];
    	buf_usb[3] = pbuf[3];
    	buf_usb[4] = pbuf[4];
    	buf_usb[5] = pbuf[5];
    	buf_usb[6] = pbuf[6];
    	buf_usb[7] = pbuf[7];
      /* Add your code here */
      break;

    case CDC_GET_LINE_CODING:
//    	memcpy((uint8_t *)pbuf, dummy, sizeof(dummy));
//      pbuf[0] = (uint8_t)(linecoding.bitrate);
//      pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
//      pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
//      pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
//      pbuf[4] = linecoding.format;
//      pbuf[5] = linecoding.paritytype;
//      pbuf[6] = linecoding.datatype;
    	pbuf[0] = buf_usb[0];
    	pbuf[1] = buf_usb[1];
    	pbuf[2] = buf_usb[2];
    	pbuf[3] = buf_usb[3];
    	pbuf[4] = buf_usb[4];
    	pbuf[5] = buf_usb[5];
    	pbuf[6] = buf_usb[6];
    	pbuf[7] = buf_usb[7];
      /* Add your code here */

      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* Add your code here */
      break;

    case CDC_SEND_BREAK:
      /* Add your code here */
      break;

    default:
      break;
  }

  return (0);
}

/**
  * @brief  Receive
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

volatile uint8_t usb_data_len = 0;
uint8_t usb_data[50] = {0};
static int8_t Receive(uint8_t *Buf, uint32_t *Len)
{
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
	usb_data_len = *Len;
	memcpy(usb_data, Buf, *Len);
	return(HAL_OK);
}




/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t Transmit(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  TransmitCplt
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  return (0);
}



/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None
  * @note   When a configuration is not supported, a default value is used.
  */
static void ComPort_Config(void)
{
	__NOP();
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

