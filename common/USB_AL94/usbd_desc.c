/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : App/usbd_desc.c
 * @version        : v1.0_Cube
 * @brief          : This file implements the USB device descriptors.
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

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usbd_core.h"

#include "usbd_composite.h"

#define USBD_LANGID_STRING 1033
#define USBD_CONFIGURATION_STRING "CONFIGURATION 0"
#define USBD_INTERFACE_STRING "COMPOSITE INTERFACE"

//#define USBD_VID                      1155
//#define USBD_MANUFACTURER_STRING      "STMicroelectronics"
//#define USBD_PID                      21156
//#define USBD_PRODUCT_STRING           "STM32 COMPOSITE DEVICE"

#define USBD_VID 0x0000 // Vendor ID for Open Source Projects is  "Generic" 0x1209
#define USBD_MANUFACTURER_STRING "VarmintWorks"
#define USBD_PID 0x0001 // pid.codes Test PID for VID=0x1209
#define USBD_PRODUCT_STRING "rosflight controller"

static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);

uint8_t * USBD_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);
uint8_t * USBD_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length);

USBD_DescriptorsTypeDef USBD_Desc = {
  USBD_DeviceDescriptor,    USBD_LangIDStrDescriptor, USBD_ManufacturerStrDescriptor, USBD_ProductStrDescriptor,
  USBD_SerialStrDescriptor, USBD_ConfigStrDescriptor, USBD_InterfaceStrDescriptor};

/** USB standard device descriptor. */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                 /*bLength */
  USB_DESC_TYPE_DEVICE, /*bDescriptorType*/
  0x00,                 /*bcdUSB */
  0x02,
  0xEF,             /*bDeviceClass*/
  0x02,             /*bDeviceSubClass*/
  0x01,             /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE, /*bMaxPacketSize*/
  LOBYTE(USBD_VID), /*idVendor*/
  HIBYTE(USBD_VID), /*idVendor*/
  LOBYTE(USBD_PID), /*idProduct*/
  HIBYTE(USBD_PID), /*idProduct*/
  0x00,             /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,          /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,      /*Index of product string*/
  USBD_IDX_SERIAL_STR,       /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION /*bNumConfigurations*/
};

/** USB lang indentifier descriptor. */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)};

/* Internal string descriptor. */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

/**
 * @brief  Return the device descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t * USBD_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

/**
 * @brief  Return the LangID string descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t * USBD_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
 * @brief  Return the product string descriptor
 * @param  speed : current device speed
 * @param  length : pointer to data length variable
 * @retval pointer to descriptor buffer
 */
uint8_t * USBD_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  if (speed == 0) {
    USBD_GetString((uint8_t *) USBD_PRODUCT_STRING, USBD_StrDesc, length);
  } else {
    USBD_GetString((uint8_t *) USBD_PRODUCT_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
 * @brief  Return the manufacturer string descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t * USBD_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *) USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
 * @brief  Return the serial number string descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t * USBD_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;

  /* Update the serial number string descriptor with the data from the unique
     * ID */
  Get_SerialNum();
  /* USER CODE BEGIN USBD_SerialStrDescriptor */

  /* USER CODE END USBD_SerialStrDescriptor */

  return (uint8_t *) USBD_StringSerial;
}

/**
 * @brief  Return the configuration string descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t * USBD_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  if (speed == USBD_SPEED_HIGH) {
    USBD_GetString((uint8_t *) USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
  } else {
    USBD_GetString((uint8_t *) USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
 * @brief  Return the interface string descriptor
 * @param  speed : Current device speed
 * @param  length : Pointer to data length variable
 * @retval Pointer to descriptor buffer
 */
uint8_t * USBD_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  if (speed == 0) {
    USBD_GetString((uint8_t *) USBD_INTERFACE_STRING, USBD_StrDesc, length);
  } else {
    USBD_GetString((uint8_t *) USBD_INTERFACE_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

#if (USBD_LPM_ENABLED == 1)
/**
  * @brief  Return the BOS descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t * length)
{
  UNUSED(speed);
  *length = sizeof(USBD_BOSDesc);
  return (uint8_t *) USBD_BOSDesc;
}
#endif /* (USBD_LPM_ENABLED == 1) */

/**
 * @brief  Create the serial number string descriptor
 * @param  None
 * @retval None
 */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0) {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

/**
 * @brief  Convert Hex 32Bits value into char
 * @param  value: value to convert
 * @param  pbuf: pointer to the buffer
 * @param  len: buffer length
 * @retval None
 */
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++) {
    if (((value >> 28)) < 0xA) {
      pbuf[2 * idx] = (value >> 28) + '0';
    } else {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
