/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_cdc_acm_if.h
 * @version        : v2.0_Cube
 * @brief          : Header for usbd_cdc_if.c file.
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

#ifndef __USBD_CDC_ACM_IF_H__
#define __USBD_CDC_ACM_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

extern USBD_CDC_ACM_ItfTypeDef USBD_CDC_ACM_fops;
uint8_t CDC_Transmit(uint8_t ch, uint8_t * Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_ACM_IF_TEMPLATE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
