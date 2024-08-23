/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usb_device.c
 * @version        : v1.0_Cube
 * @brief          : This file implements the USB Device
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_desc.h"

#include "usbd_composite.h"

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDevice;

/**
 * Init USB device Library, add supported class and start the library
 * @retval None
 */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  USBD_COMPOSITE_Mount_Class();

#if (USBD_USE_HS == 1)
  if (USBD_Init(&hUsbDevice, &USBD_Desc, DEVICE_HS) != USBD_OK) { Error_Handler(); }
#else
  if (USBD_Init(&hUsbDevice, &USBD_Desc, DEVICE_FS) != USBD_OK) { Error_Handler(); }
#endif

  if (USBD_RegisterClass(&hUsbDevice, &USBD_COMPOSITE) != USBD_OK) { Error_Handler(); }

  if (USBD_CDC_ACM_RegisterInterface(&hUsbDevice, &USBD_CDC_ACM_fops) != USBD_OK) { Error_Handler(); }

  if (USBD_Start(&hUsbDevice) != USBD_OK) { Error_Handler(); }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
