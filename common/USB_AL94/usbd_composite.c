/**
 ******************************************************************************
 * @file    usbd_composite.c
 * @author  MCD Application Team
 * @brief   This file provides the HID core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                COMPOSITE Class  Description
 *          ===================================================================
 *
 *
 *
 *
 *
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 *
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

#include "usbd_composite.h"
#include "usbd_ctlreq.h"

static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef * pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef * pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef * pdev, USBD_SetupReqTypedef * req);
static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef * pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef * pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef * pdev);
static uint8_t USBD_COMPOSITE_EP0_TxReady(USBD_HandleTypeDef * pdev);
static uint8_t USBD_COMPOSITE_SOF(USBD_HandleTypeDef * pdev);
static uint8_t USBD_COMPOSITE_IsoINIncomplete(USBD_HandleTypeDef * pdev, uint8_t epnum);
static uint8_t USBD_COMPOSITE_IsoOutIncomplete(USBD_HandleTypeDef * pdev, uint8_t epnum);

static uint8_t * USBD_COMPOSITE_GetHSCfgDesc(uint16_t * length);
static uint8_t * USBD_COMPOSITE_GetFSCfgDesc(uint16_t * length);
static uint8_t * USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t * length);
static uint8_t * USBD_COMPOSITE_GetDeviceQualifierDesc(uint16_t * length);
static uint8_t * USBD_COMPOSITE_GetUsrStringDesc(USBD_HandleTypeDef * pdev, uint8_t index, uint16_t * length);

USBD_ClassTypeDef USBD_COMPOSITE = {USBD_COMPOSITE_Init,
                                    USBD_COMPOSITE_DeInit,
                                    USBD_COMPOSITE_Setup,
                                    USBD_COMPOSITE_EP0_TxReady,
                                    USBD_COMPOSITE_EP0_RxReady,
                                    USBD_COMPOSITE_DataIn,
                                    USBD_COMPOSITE_DataOut,
                                    USBD_COMPOSITE_SOF,
                                    USBD_COMPOSITE_IsoINIncomplete,
                                    USBD_COMPOSITE_IsoOutIncomplete,
                                    USBD_COMPOSITE_GetHSCfgDesc,
                                    USBD_COMPOSITE_GetFSCfgDesc,
                                    USBD_COMPOSITE_GetOtherSpeedCfgDesc,
                                    USBD_COMPOSITE_GetDeviceQualifierDesc,
                                    USBD_COMPOSITE_GetUsrStringDesc};

typedef struct USBD_COMPOSITE_CFG_DESC_t
{
  uint8_t CONFIG_DESC[USB_CONF_DESC_SIZE];

  uint8_t USBD_CDC_ACM_DESC[USB_CDC_CONFIG_DESC_SIZ - 0x09];

} __PACKED USBD_COMPOSITE_CFG_DESC_t;

__ALIGN_BEGIN USBD_COMPOSITE_CFG_DESC_t USBD_COMPOSITE_FSCfgDesc, USBD_COMPOSITE_HSCfgDesc __ALIGN_END;
uint8_t USBD_Track_String_Index = (USBD_IDX_INTERFACE_STR + 1);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
  USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0xEF, 0x02, 0x01, 0x40, 0x01, 0x00,
};

/**
 * @brief  USBD_COMPOSITE_Init
 *         Initialize the COMPOSITE interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_COMPOSITE_Init(USBD_HandleTypeDef * pdev, uint8_t cfgidx)
{
  USBD_CDC_ACM.Init(pdev, cfgidx);
  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_Init
 *         DeInitialize the COMPOSITE layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_COMPOSITE_DeInit(USBD_HandleTypeDef * pdev, uint8_t cfgidx)
{
  USBD_CDC_ACM.DeInit(pdev, cfgidx);

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_Setup
 *         Handle the COMPOSITE specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_COMPOSITE_Setup(USBD_HandleTypeDef * pdev, USBD_SetupReqTypedef * req)
{
  for (uint8_t i = 0; i < USBD_CDC_ACM_COUNT; i++) {
    if (LOBYTE(req->wIndex) == CDC_CMD_ITF_NBR[i] || LOBYTE(req->wIndex) == CDC_COM_ITF_NBR[i]) {
      return USBD_CDC_ACM.Setup(pdev, req);
    }
  }

  return USBD_FAIL;
}

/**
 * @brief  USBD_COMPOSITE_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_COMPOSITE_DataIn(USBD_HandleTypeDef * pdev, uint8_t epnum)
{
  for (uint8_t i = 0; i < USBD_CDC_ACM_COUNT; i++) {
    if (epnum == (CDC_IN_EP[i] & 0x7F) || epnum == (CDC_CMD_EP[i] & 0x7F)) { return USBD_CDC_ACM.DataIn(pdev, epnum); }
  }
  return USBD_FAIL;
}

/**
 * @brief  USBD_COMPOSITE_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_COMPOSITE_EP0_RxReady(USBD_HandleTypeDef * pdev)
{
  USBD_CDC_ACM.EP0_RxReady(pdev);
  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_EP0_TxReady
 *         handle EP0 TRx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_COMPOSITE_EP0_TxReady(USBD_HandleTypeDef * pdev) { return (uint8_t) USBD_OK; }

/**
 * @brief  USBD_COMPOSITE_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_COMPOSITE_SOF(USBD_HandleTypeDef * pdev) { return (uint8_t) USBD_OK; }

/**
 * @brief  USBD_COMPOSITE_IsoINIncomplete
 *         handle data ISO IN Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_COMPOSITE_IsoINIncomplete(USBD_HandleTypeDef * pdev, uint8_t epnum) { return (uint8_t) USBD_OK; }

/**
 * @brief  USBD_COMPOSITE_IsoOutIncomplete
 *         handle data ISO OUT Incomplete event
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_COMPOSITE_IsoOutIncomplete(USBD_HandleTypeDef * pdev, uint8_t epnum) { return (uint8_t) USBD_OK; }
/**
 * @brief  USBD_COMPOSITE_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_COMPOSITE_DataOut(USBD_HandleTypeDef * pdev, uint8_t epnum)
{
  for (uint8_t i = 0; i < USBD_CDC_ACM_COUNT; i++) {
    if (epnum == CDC_OUT_EP[i]) { return USBD_CDC_ACM.DataOut(pdev, epnum); }
  }
  return USBD_FAIL;
}

/**
 * @brief  USBD_COMPOSITE_GetHSCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t * USBD_COMPOSITE_GetHSCfgDesc(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_COMPOSITE_HSCfgDesc);
  return (uint8_t *) &USBD_COMPOSITE_HSCfgDesc;
}

/**
 * @brief  USBD_COMPOSITE_GetFSCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t * USBD_COMPOSITE_GetFSCfgDesc(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_COMPOSITE_FSCfgDesc);
  return (uint8_t *) &USBD_COMPOSITE_FSCfgDesc;
}

/**
 * @brief  USBD_COMPOSITE_GetOtherSpeedCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t * USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t * length)
{
#if (USBD_USE_HS == 1)
  *length = (uint16_t) sizeof(USBD_COMPOSITE_FSCfgDesc);
  return (uint8_t *) &USBD_COMPOSITE_FSCfgDesc;
#else
  *length = (uint16_t) sizeof(USBD_COMPOSITE_HSCfgDesc);
  return (uint8_t *) &USBD_COMPOSITE_HSCfgDesc;
#endif
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t * USBD_COMPOSITE_GetDeviceQualifierDesc(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_COMPOSITE_DeviceQualifierDesc);
  return USBD_COMPOSITE_DeviceQualifierDesc;
}

/**
 * @brief  USBD_COMPOSITE_GetUsrStringDesc
 *         Manages the transfer of memory interfaces string descriptors.
 * @param  speed : current device speed
 * @param  index: descriptor index
 * @param  length : pointer data length
 * @retval pointer to the descriptor table or NULL if the descriptor is not supported.
 */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t * USBD_COMPOSITE_GetUsrStringDesc(USBD_HandleTypeDef * pdev, uint8_t index, uint16_t * length)
{
  static uint8_t USBD_StrDesc[64];

  /* Check if the requested string interface is supported */
  if (index <= USBD_Track_String_Index) {
    char str_buffer[16] = "";
    for (uint8_t i = 0; i < USBD_CDC_ACM_COUNT; i++) {
      if (index == CDC_STR_DESC_IDX[i]) {
        snprintf(str_buffer, sizeof(str_buffer), CDC_ACM_STR_DESC, i);
        USBD_GetString((uint8_t *) str_buffer, USBD_StrDesc, length);
      }
    }
    return USBD_StrDesc;
  } else {
    /* Not supported Interface Descriptor index */
    return NULL;
  }
}
#endif

void USBD_COMPOSITE_Mount_Class(void)
{
  uint16_t len = 0;
  uint8_t * ptr = NULL;

  uint8_t in_ep_track = 0x81;
  uint8_t out_ep_track = 0x01;
  uint8_t interface_no_track = 0x00;

  ptr = USBD_CDC_ACM.GetFSConfigDescriptor(&len);
  USBD_Update_CDC_ACM_DESC(ptr, interface_no_track, interface_no_track + 1, in_ep_track, in_ep_track + 1, out_ep_track,
                           USBD_Track_String_Index);
  memcpy(USBD_COMPOSITE_FSCfgDesc.USBD_CDC_ACM_DESC, ptr + 0x09, len - 0x09);

  ptr = USBD_CDC_ACM.GetHSConfigDescriptor(&len);
  USBD_Update_CDC_ACM_DESC(ptr, interface_no_track, interface_no_track + 1, in_ep_track, in_ep_track + 1, out_ep_track,
                           USBD_Track_String_Index);
  memcpy(USBD_COMPOSITE_HSCfgDesc.USBD_CDC_ACM_DESC, ptr + 0x09, len - 0x09);

  in_ep_track += 2 * USBD_CDC_ACM_COUNT;
  out_ep_track += 1 * USBD_CDC_ACM_COUNT;
  interface_no_track += 2 * USBD_CDC_ACM_COUNT;
  USBD_Track_String_Index += USBD_CDC_ACM_COUNT;

  uint16_t CFG_SIZE = sizeof(USBD_COMPOSITE_CFG_DESC_t);
  ptr = USBD_COMPOSITE_HSCfgDesc.CONFIG_DESC;
  /* Configuration Descriptor */
  ptr[0] = 0x09;                        /* bLength: Configuration Descriptor size */
  ptr[1] = USB_DESC_TYPE_CONFIGURATION; /* bDescriptorType: Configuration */
  ptr[2] = LOBYTE(CFG_SIZE);            /* wTotalLength:no of returned bytes */
  ptr[3] = HIBYTE(CFG_SIZE);
  ptr[4] = interface_no_track; /* bNumInterfaces: 2 interface */
  ptr[5] = 0x01;               /* bConfigurationValue: Configuration value */
  ptr[6] = 0x00;               /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  ptr[7] = 0xC0; /* bmAttributes: Bus Powered according to user configuration */
#else
  ptr[7] = 0x80; /* bmAttributes: Bus Powered according to user configuration */
#endif
  ptr[8] = USBD_MAX_POWER; /* MaxPower 100 mA */

  ptr = USBD_COMPOSITE_FSCfgDesc.CONFIG_DESC;
  /* Configuration Descriptor */
  ptr[0] = 0x09;                        /* bLength: Configuration Descriptor size */
  ptr[1] = USB_DESC_TYPE_CONFIGURATION; /* bDescriptorType: Configuration */
  ptr[2] = LOBYTE(CFG_SIZE);            /* wTotalLength:no of returned bytes */
  ptr[3] = HIBYTE(CFG_SIZE);
  ptr[4] = interface_no_track; /* bNumInterfaces: 2 interface */
  ptr[5] = 0x01;               /* bConfigurationValue: Configuration value */
  ptr[6] = 0x00;               /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  ptr[7] = 0xC0; /* bmAttributes: Bus Powered according to user configuration */
#else
  ptr[7] = 0x80; /* bmAttributes: Bus Powered according to user configuration */
#endif
  ptr[8] = USBD_MAX_POWER; /* MaxPower 100 mA */

  (void) out_ep_track;
  (void) in_ep_track;
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
