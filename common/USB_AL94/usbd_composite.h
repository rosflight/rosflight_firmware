/**
 ******************************************************************************
 * @file    usbd_composite.h
 * @author  MCD Application Team
 * @brief   Header file for the usbd_composite.c file.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      www.st.com/SLA0044
 *
 ******************************************************************************
 */

#ifndef __USB_COMPOSITE_H
#define __USB_COMPOSITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "usbd_ioreq.h"
#include <BoardConfig.h> // NOTE! replaced "AL94.I-CUBE-USBD-COMPOSITE_conf.h"

#define USBD_USE_HS _USBD_USE_HS
#define USBD_USE_CDC_ACM _USBD_USE_CDC_ACM
#define USBD_CDC_ACM_COUNT _USBD_CDC_ACM_COUNT
//#define USBD_USE_CDC_RNDIS           _USBD_USE_CDC_RNDIS
//#define USBD_USE_CDC_ECM             _USBD_USE_CDC_ECM
//#define USBD_USE_HID_MOUSE           _USBD_USE_HID_MOUSE
//#define USBD_USE_HID_KEYBOARD        _USBD_USE_HID_KEYBOARD
//#define USBD_USE_HID_CUSTOM          _USBD_USE_HID_CUSTOM
//#define USBD_USE_UAC_MIC             _USBD_USE_UAC_MIC
//#define USBD_USE_UAC_SPKR            _USBD_USE_UAC_SPKR
//#define USBD_USE_UVC                 _USBD_USE_UVC
//#define USBD_USE_MSC                 _USBD_USE_MSC
//#define USBD_USE_DFU                 _USBD_USE_DFU
//#define USBD_USE_PRNTR               _USBD_USE_PRNTR

#include "usbd_cdc_if.h"

extern USBD_ClassTypeDef USBD_COMPOSITE;

void USBD_COMPOSITE_Mount_Class(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_COMPOSITE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
