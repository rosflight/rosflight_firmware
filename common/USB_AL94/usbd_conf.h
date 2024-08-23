/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_conf.h
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

#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*---------- -----------*/
#define USBD_MAX_NUM_INTERFACES 15U
/*---------- -----------*/
#define USBD_MAX_NUM_CONFIGURATION 1U
/*---------- -----------*/
#define USBD_MAX_STR_DESC_SIZ 512U
/*---------- -----------*/
#define USBD_SUPPORT_USER_STRING_DESC 1U
/*---------- -----------*/
#define USBD_DEBUG_LEVEL 0U
/*---------- -----------*/
#define USBD_LPM_ENABLED 0U
/*---------- -----------*/
#define USBD_SELF_POWERED 1U
/*---------- -----------*/

/****************************************/
/* #define for FS and HS identification */
#define DEVICE_FS 0
#define DEVICE_HS 1

/* Memory management macros */

/** Alias for memory allocation. */
#define USBD_malloc malloc

/** Alias for memory release. */
#define USBD_free free

/** Alias for memory set. */
#define USBD_memset memset

/** Alias for memory copy. */
#define USBD_memcpy memcpy

/** Alias for delay. */
#define USBD_Delay HAL_Delay

/* DEBUG macros */

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)                                                                                               \
  printf(__VA_ARGS__);                                                                                                 \
  printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)

#define USBD_ErrLog(...)                                                                                               \
  printf("ERROR: ");                                                                                                   \
  printf(__VA_ARGS__);                                                                                                 \
  printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)                                                                                               \
  printf("DEBUG : ");                                                                                                  \
  printf(__VA_ARGS__);                                                                                                 \
  printf("\n");
#else
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
