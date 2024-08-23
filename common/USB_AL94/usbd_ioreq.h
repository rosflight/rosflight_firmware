/**
 ******************************************************************************
 * @file    usbd_ioreq.h
 * @author  MCD Application Team
 * @brief   Header file for the usbd_ioreq.c file
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

#ifndef __USBD_IOREQ_H
#define __USBD_IOREQ_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_core.h"
#include "usbd_def.h"

USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef * pdev, uint8_t * pbuf, uint32_t len);

USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef * pdev, uint8_t * pbuf, uint32_t len);

USBD_StatusTypeDef USBD_CtlPrepareRx(USBD_HandleTypeDef * pdev, uint8_t * pbuf, uint32_t len);

USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef * pdev, uint8_t * pbuf, uint32_t len);

USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef * pdev);
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef * pdev);

uint32_t USBD_GetRxCount(USBD_HandleTypeDef * pdev, uint8_t ep_addr);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_IOREQ_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
