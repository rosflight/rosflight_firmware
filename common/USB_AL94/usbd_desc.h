/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_desc.c
 * @version        : v1.0_Cube
 * @brief          : Header for usbd_conf.c file.
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

#ifndef __USBD_DESC__C__
#define __USBD_DESC__C__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_def.h"
/** @defgroup USBD_DESC_Exported_Constants USBD_DESC_Exported_Constants
     * @brief Constants.
     * @{
     */
#define DEVICE_ID1 (UID_BASE)
#define DEVICE_ID2 (UID_BASE + 0x4)
#define DEVICE_ID3 (UID_BASE + 0x8)

#define USB_SIZ_STRING_SERIAL 0x1A

extern USBD_DescriptorsTypeDef USBD_Desc;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC__C__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
