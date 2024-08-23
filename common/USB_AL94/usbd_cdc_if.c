/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_if.c
 * @version        : v2.0_Cube
 * @brief          : Usb device for Virtual Com Port.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

#include <Callbacks.h>

#define APP_RX_DATA_SIZE 128
//#define APP_TX_DATA_SIZE 128

/** RX buffer for USB */
uint8_t RX_Buffer[NUMBER_OF_CDC][APP_RX_DATA_SIZE];

/** TX buffer for USB, RX buffer for UART */
//uint8_t TX_Buffer[NUMBER_OF_CDC][APP_TX_DATA_SIZE];
USBD_CDC_ACM_LineCodingTypeDef Line_Coding[NUMBER_OF_CDC];

uint32_t Write_Index[NUMBER_OF_CDC]; /* keep track of received data over UART */
uint32_t Read_Index[NUMBER_OF_CDC];  /* keep track of sent data to USB */

extern USBD_HandleTypeDef hUsbDevice;

static int8_t CDC_Init(uint8_t cdc_ch);
static int8_t CDC_DeInit(uint8_t cdc_ch);
static int8_t CDC_Control(uint8_t cdc_ch, uint8_t cmd, uint8_t * pbuf, uint16_t length);
static int8_t CDC_Receive(uint8_t cdc_ch, uint8_t * pbuf, uint32_t * Len);
static int8_t CDC_TransmitCplt(uint8_t cdc_ch, uint8_t * Buf, uint32_t * Len, uint8_t epnum);

USBD_CDC_ACM_ItfTypeDef USBD_CDC_ACM_fops = {CDC_Init, CDC_DeInit, CDC_Control, CDC_Receive, CDC_TransmitCplt};

static int8_t CDC_Init(uint8_t cdc_ch)
{
  /* USER CODE BEGIN 3 */

  /* ##-1- Set Application Buffers */
  USBD_CDC_SetRxBuffer(cdc_ch, &hUsbDevice, RX_Buffer[cdc_ch]);

  //  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  //  /* Start Channel1 */
  //  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  //  {
  //    /* Starting Error */
  //    Error_Handler();
  //  }

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit(uint8_t cdc_ch)
{
  /* USER CODE BEGIN 4 */
  /* DeInitialize the UART peripheral */
  //  if (HAL_UART_DeInit(CDC_CH_To_UART_Handle(cdc_ch)) != HAL_OK)
  //  {
  //    /* Initialization Error */
  //    Error_Handler();
  //  }
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Control(uint8_t cdc_ch, uint8_t cmd, uint8_t * pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch (cmd) {
    case CDC_SEND_ENCAPSULATED_COMMAND:

      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

      break;

    case CDC_SET_COMM_FEATURE:

      break;

    case CDC_GET_COMM_FEATURE:

      break;

    case CDC_CLEAR_COMM_FEATURE:

      break;

      /*******************************************************************************/
      /* Line Coding Structure                                                       */
      /*-----------------------------------------------------------------------------*/
      /* Offset | Field       | Size | Value  | Description                          */
      /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
      /* 4      | bCharFormat |   1  | Number | Stop bits                            */
      /*                                        0 - 1 Stop bit                       */
      /*                                        1 - 1.5 Stop bits                    */
      /*                                        2 - 2 Stop bits                      */
      /* 5      | bParityType |  1   | Number | Parity                               */
      /*                                        0 - None                             */
      /*                                        1 - Odd                              */
      /*                                        2 - Even                             */
      /*                                        3 - Mark                             */
      /*                                        4 - Space                            */
      /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
      /*******************************************************************************/
    case CDC_SET_LINE_CODING:
      Line_Coding[cdc_ch].bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
      Line_Coding[cdc_ch].format = pbuf[4];
      Line_Coding[cdc_ch].paritytype = pbuf[5];
      Line_Coding[cdc_ch].datatype = pbuf[6];

      //Change_UART_Setting(cdc_ch);
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t) (Line_Coding[cdc_ch].bitrate);
      pbuf[1] = (uint8_t) (Line_Coding[cdc_ch].bitrate >> 8);
      pbuf[2] = (uint8_t) (Line_Coding[cdc_ch].bitrate >> 16);
      pbuf[3] = (uint8_t) (Line_Coding[cdc_ch].bitrate >> 24);
      pbuf[4] = Line_Coding[cdc_ch].format;
      pbuf[5] = Line_Coding[cdc_ch].paritytype;
      pbuf[6] = Line_Coding[cdc_ch].datatype;
      break;

    case CDC_SET_CONTROL_LINE_STATE:

      break;

    case CDC_SEND_BREAK:

      break;

    default:
      break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
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
static int8_t CDC_Receive(uint8_t cdc_ch, uint8_t * Buf, uint32_t * Len)
{
  /* USER CODE BEGIN 6 */
  CDC_Receive_Callback(cdc_ch, Buf, *Len); //NOTE! AL94 USB CDC. We implement this.

  USBD_CDC_SetRxBuffer(cdc_ch, &hUsbDevice, &Buf[0]);
  USBD_CDC_ReceivePacket(cdc_ch, &hUsbDevice);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
 * @brief  CDC_TransmitCplt_FS
 *         Data transmited callback
 *
 *         @note
 *         This function is IN transfer complete callback used to inform user that
 *         the submitted Data is successfully sent over USB.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_TransmitCplt(uint8_t cdc_ch, uint8_t * Buf, uint32_t * Len, uint8_t epnum)
{
  CDC_TransmitCplt_Callback(cdc_ch, Buf, (uint16_t) (*Len)); //NOTE! AL94 USB CDC. We implement this.

  return (USBD_OK);
}

/**
 * @brief  CDC_Transmit
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit(uint8_t ch, uint8_t * Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  extern USBD_CDC_ACM_HandleTypeDef CDC_ACM_Class_Data[];
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;
  hcdc = &CDC_ACM_Class_Data[ch];
  if (hcdc->TxState != 0) { return USBD_BUSY; }
  USBD_CDC_SetTxBuffer(ch, &hUsbDevice, Buf, Len);
  result = USBD_CDC_TransmitPacket(ch, &hUsbDevice);
  /* USER CODE END 7 */
  return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
