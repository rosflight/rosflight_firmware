/**
 ******************************************************************************
 * @file    usbd_cdc.c
 * @author  MCD Application Team
 * @brief   This file provides the high layer firmware functions to manage the
 *          following functionalities of the USB CDC Class:
 *           - Initialization and Configuration of high and low layer
 *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
 *           - OUT/IN data transfer
 *           - Command IN transfer (class requests management)
 *           - Error management
 *
 *  @verbatim
 *
 *          ===================================================================
 *                                CDC Class Driver Description
 *          ===================================================================
 *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
 *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
 *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
 *           This driver implements the following aspects of the specification:
 *             - Device descriptor management
 *             - Configuration descriptor management
 *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
 *             - Requests management (as described in section 6.2 in specification)
 *             - Abstract Control Model compliant
 *             - Union Functional collection (using 1 IN endpoint for control)
 *             - Data interface class
 *
 *           These aspects may be enriched or modified for a specific user application.
 *
 *            This driver doesn't implement the following aspects of the specification
 *            (but it is possible to manage these features with some modifications on this driver):
 *             - Any class-specific aspect relative to communication classes should be managed by user application.
 *             - All communication classes other than PSTN are not managed
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

#include "usbd_cdc.h"
#include "usbd_ctlreq.h"

#define _CDC_IN_EP 0x81U  /* EP1 for data IN */
#define _CDC_OUT_EP 0x01U /* EP1 for data OUT */
#define _CDC_CMD_EP 0x82U /* EP2 for CDC commands */

#define _CDC_CMD_ITF_NBR 0x00U /* Command Interface Number */
#define _CDC_COM_ITF_NBR 0x01U /* Communication Interface Number */

#define _CDC_STR_DESC_IDX 0x00

uint8_t CDC_IN_EP[NUMBER_OF_CDC];
uint8_t CDC_OUT_EP[NUMBER_OF_CDC];
uint8_t CDC_CMD_EP[NUMBER_OF_CDC];

uint8_t CDC_CMD_ITF_NBR[NUMBER_OF_CDC];
uint8_t CDC_COM_ITF_NBR[NUMBER_OF_CDC];

uint8_t CDC_STR_DESC_IDX[NUMBER_OF_CDC];

/** @defgroup USBD_CDC_Private_FunctionPrototypes
 * @{
 */

static uint8_t USBD_CDC_Init(USBD_HandleTypeDef * pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef * pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef * pdev, USBD_SetupReqTypedef * req);
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef * pdev, uint8_t epnum);
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef * pdev, uint8_t epnum);
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef * pdev);

static uint8_t * USBD_CDC_GetFSCfgDesc(uint16_t * length);
static uint8_t * USBD_CDC_GetHSCfgDesc(uint16_t * length);
static uint8_t * USBD_CDC_GetOtherSpeedCfgDesc(uint16_t * length);
static uint8_t * USBD_CDC_GetOtherSpeedCfgDesc(uint16_t * length);
static uint8_t * USBD_CDC_GetDeviceQualifierDescriptor(uint16_t * length);

USBD_CDC_ACM_HandleTypeDef CDC_ACM_Class_Data[NUMBER_OF_CDC];

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
  USB_LEN_DEV_QUALIFIER_DESC, USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
};

/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Variables
 * @{
 */

/* CDC interface class callbacks structure */
USBD_ClassTypeDef USBD_CDC_ACM = {
  USBD_CDC_Init,
  USBD_CDC_DeInit,
  USBD_CDC_Setup,
  NULL, /* EP0_TxSent, */
  USBD_CDC_EP0_RxReady,
  USBD_CDC_DataIn,
  USBD_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_CDC_GetHSCfgDesc,
  USBD_CDC_GetFSCfgDesc,
  USBD_CDC_GetOtherSpeedCfgDesc,
  USBD_CDC_GetDeviceQualifierDescriptor,
};

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END = {
  /* Configuration Descriptor */
  0x09,                                                 /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                          /* bDescriptorType: Configuration */
  LOBYTE(USB_CDC_CONFIG_DESC_SIZ),                      /* wTotalLength:no of returned bytes */
  HIBYTE(USB_CDC_CONFIG_DESC_SIZ), (2 * NUMBER_OF_CDC), /* bNumInterfaces: 2 interface */
  0x01,                                                 /* bConfigurationValue: Configuration value */
  0x00, /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0, /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80, /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER, /* MaxPower 100 mA */

  /********************  CDC0 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

#if (NUMBER_OF_CDC > 1)
  /********************  CDC1 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 2)
  /********************  CDC2 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 3)
  /********************  CDC3 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 4)
  /********************  CDC4 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 5)
  /********************  CDC5 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 6)
  /********************  CDC6 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 7)
  /********************  CDC7 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  _CDC_CMD_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x01,                    /* bNumEndpoints: One endpoints used */
  0x02,                    /* bInterfaceClass: Communication Interface Class */
  0x02,                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                    /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX,       /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_HS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                               /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),      /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), 0x00 /* bInterval: ignore for Bulk transfer */
#endif
};

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END = {
  /* Configuration Descriptor */
  0x09,                                  /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,           /* bDescriptorType: Configuration */
  LOBYTE(USB_CDC_CONFIG_DESC_SIZ),       /* wTotalLength:no of returned bytes */
  HIBYTE(USB_CDC_CONFIG_DESC_SIZ), 0x02, /* bNumInterfaces: 2 interface */
  0x01,                                  /* bConfigurationValue: Configuration value */
  0x00,                                  /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0, /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80, /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER, /* MaxPower 100 mA */

  /*---------------------------------------------------------------------------*/
  /********************  CDC0 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

#if (NUMBER_OF_CDC > 1)
  /********************  CDC1 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 2)
  /********************  CDC2 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 3)
  /********************  CDC3 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 4)
  /********************  CDC4 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 5)
  /********************  CDC5 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 6)
  /********************  CDC6 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                                /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */
#endif

#if (NUMBER_OF_CDC > 7)
  /********************  CDC7 block ********************/
  /******** IAD to associate the two CDC interfaces */
  0x08,             /* bLength */
  0x0B,             /* bDescriptorType */
  _CDC_CMD_ITF_NBR, /* bFirstInterface */
  0x02,             /* bInterfaceCount */
  0x02,             /* bFunctionClass */
  0x02,             /* bFunctionSubClass */
  0x01,             /* bFunctionProtocol */
  0x00,             /* iFunction (Index of string descriptor describing this function) */

  /* Interface Descriptor */
  0x09,                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface */
  /* Interface descriptor type */
  _CDC_CMD_ITF_NBR,  /* bInterfaceNumber: Number of Interface */
  0x00,              /* bAlternateSetting: Alternate setting */
  0x01,              /* bNumEndpoints: One endpoints used */
  0x02,              /* bInterfaceClass: Communication Interface Class */
  0x02,              /* bInterfaceSubClass: Abstract Control Model */
  0x01,              /* bInterfaceProtocol: Common AT commands */
  _CDC_STR_DESC_IDX, /* iInterface: */

  /* Header Functional Descriptor */
  0x05, /* bLength: Endpoint Descriptor size */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x00, /* bDescriptorSubtype: Header Func Desc */
  0x10, /* bcdCDC: spec release number */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x01,             /* bDescriptorSubtype: Call Management Func Desc */
  0x00,             /* bmCapabilities: D0+D1 */
  _CDC_COM_ITF_NBR, /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x02, /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,             /* bFunctionLength */
  0x24,             /* bDescriptorType: CS_INTERFACE */
  0x06,             /* bDescriptorSubtype: Union func desc */
  _CDC_CMD_ITF_NBR, /* bMasterInterface: Communication class interface */
  _CDC_COM_ITF_NBR, /* bSlaveInterface0: Data Class Interface */

  /* Endpoint 2 Descriptor */
  0x07,                                          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType: Endpoint */
  _CDC_CMD_EP,                                   /* bEndpointAddress */
  0x03,                                          /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),                   /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE), CDC_FS_BINTERVAL, /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /* Data class interface descriptor */
  0x09,                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: */
  _CDC_COM_ITF_NBR,        /* bInterfaceNumber: Number of Interface */
  0x00,                    /* bAlternateSetting: Alternate setting */
  0x02,                    /* bNumEndpoints: Two endpoints used */
  0x0A,                    /* bInterfaceClass: CDC */
  0x00,                    /* bInterfaceSubClass: */
  0x00,                    /* bInterfaceProtocol: */
  0x00,                    /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                      /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType: Endpoint */
  _CDC_OUT_EP,                               /* bEndpointAddress */
  0x02,                                      /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),       /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00, /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  _CDC_IN_EP,                               /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),      /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE), 0x00 /* bInterval: ignore for Bulk transfer */
#endif
};

/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Functions
 * @{
 */

/**
 * @brief  USBD_CDC_Init
 *         Initialize the CDC interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_CDC_Init(USBD_HandleTypeDef * pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;

  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    hcdc = &CDC_ACM_Class_Data[i];

    if (pdev->dev_speed == USBD_SPEED_HIGH) {
      /* Open EP IN */
      (void) USBD_LL_OpenEP(pdev, CDC_IN_EP[i], USBD_EP_TYPE_BULK, CDC_DATA_HS_IN_PACKET_SIZE);

      pdev->ep_in[CDC_IN_EP[i] & 0xFU].is_used = 1U;

      /* Open EP OUT */
      (void) USBD_LL_OpenEP(pdev, CDC_OUT_EP[i], USBD_EP_TYPE_BULK, CDC_DATA_HS_OUT_PACKET_SIZE);

      pdev->ep_out[CDC_OUT_EP[i] & 0xFU].is_used = 1U;

      /* Set bInterval for CDC CMD Endpoint */
      pdev->ep_in[CDC_CMD_EP[i] & 0xFU].bInterval = CDC_HS_BINTERVAL;
    } else {
      /* Open EP IN */
      (void) USBD_LL_OpenEP(pdev, CDC_IN_EP[i], USBD_EP_TYPE_BULK, CDC_DATA_FS_IN_PACKET_SIZE);

      pdev->ep_in[CDC_IN_EP[i] & 0xFU].is_used = 1U;

      /* Open EP OUT */
      (void) USBD_LL_OpenEP(pdev, CDC_OUT_EP[i], USBD_EP_TYPE_BULK, CDC_DATA_FS_OUT_PACKET_SIZE);

      pdev->ep_out[CDC_OUT_EP[i] & 0xFU].is_used = 1U;

      /* Set bInterval for CMD Endpoint */
      pdev->ep_in[CDC_CMD_EP[i] & 0xFU].bInterval = CDC_FS_BINTERVAL;
    }

    /* Open Command IN EP */
    (void) USBD_LL_OpenEP(pdev, CDC_CMD_EP[i], USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
    pdev->ep_in[CDC_CMD_EP[i] & 0xFU].is_used = 1U;

    /* Init  physical Interface components */
    ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)->Init(i);

    /* Init Xfer states */
    hcdc->TxState = 0U;
    hcdc->RxState = 0U;

    if (pdev->dev_speed == USBD_SPEED_HIGH) {
      /* Prepare Out endpoint to receive next packet */
      (void) USBD_LL_PrepareReceive(pdev, CDC_OUT_EP[i], hcdc->RxBuffer, CDC_DATA_HS_OUT_PACKET_SIZE);
    } else {
      /* Prepare Out endpoint to receive next packet */
      (void) USBD_LL_PrepareReceive(pdev, CDC_OUT_EP[i], hcdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);
    }
  }
  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_Init
 *         DeInitialize the CDC layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef * pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    /* Close EP IN */
    (void) USBD_LL_CloseEP(pdev, CDC_IN_EP[i]);
    pdev->ep_in[CDC_IN_EP[i] & 0xFU].is_used = 0U;

    /* Close EP OUT */
    (void) USBD_LL_CloseEP(pdev, CDC_OUT_EP[i]);
    pdev->ep_out[CDC_OUT_EP[i] & 0xFU].is_used = 0U;

    /* Close Command IN EP */
    (void) USBD_LL_CloseEP(pdev, CDC_CMD_EP[i]);
    pdev->ep_in[CDC_CMD_EP[i] & 0xFU].is_used = 0U;
    pdev->ep_in[CDC_CMD_EP[i] & 0xFU].bInterval = 0U;

    /* DeInit  physical Interface components */
    ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)->DeInit(i);
  }
  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_Setup
 *         Handle the CDC specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_CDC_Setup(USBD_HandleTypeDef * pdev, USBD_SetupReqTypedef * req)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;
  uint16_t len;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  uint8_t windex_to_ch = 0;

  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    if (LOBYTE(req->wIndex) == CDC_CMD_ITF_NBR[i] || LOBYTE(req->wIndex) == CDC_COM_ITF_NBR[i]) {
      windex_to_ch = i;
      break;
    }
  }

  hcdc = &CDC_ACM_Class_Data[windex_to_ch];

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    case USB_REQ_TYPE_CLASS:
      if (req->wLength != 0U) {
        if ((req->bmRequest & 0x80U) != 0U) {
          ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)
            ->Control(windex_to_ch, req->bRequest, (uint8_t *) hcdc->data[windex_to_ch], req->wLength);

          len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
          (void) USBD_CtlSendData(pdev, (uint8_t *) hcdc->data[windex_to_ch], len);
        } else {
          hcdc->CmdOpCode = req->bRequest;
          hcdc->CmdLength = (uint8_t) req->wLength;

          (void) USBD_CtlPrepareRx(pdev, (uint8_t *) hcdc->data[windex_to_ch], req->wLength);
        }
      } else {
        ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)
          ->Control(windex_to_ch, req->bRequest, (uint8_t *) req, 0U);
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            (void) USBD_CtlSendData(pdev, (uint8_t *) &status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            (void) USBD_CtlSendData(pdev, &ifalt, 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state != USBD_STATE_CONFIGURED) {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t) ret;
}

/**
 * @brief  USBD_CDC_DataIn
 *         Data sent on non-control IN endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef * pdev, uint8_t epnum)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;
  PCD_HandleTypeDef * hpcd = pdev->pData;
  uint8_t ep_to_ch = 0;

  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    if (epnum == (CDC_IN_EP[i] & 0x0F)) {
      ep_to_ch = i;
      break;
    }
  }

  hcdc = &CDC_ACM_Class_Data[ep_to_ch];

  if ((pdev->ep_in[epnum].total_length > 0U)
      && ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U)) {
    /* Update the packet total length */
    pdev->ep_in[epnum].total_length = 0U;

    /* Send ZLP */
    (void) USBD_LL_Transmit(pdev, epnum, NULL, 0U);
  } else {
    hcdc->TxState = 0U;

    if (((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)->TransmitCplt != NULL) {
      ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)
        ->TransmitCplt(ep_to_ch, hcdc->TxBuffer, &hcdc->TxLength, epnum);
    }
  }

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_DataOut
 *         Data received on non-control Out endpoint
 * @param  pdev: device instance
 * @param  epnum: endpoint number
 * @retval status
 */

static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef * pdev, uint8_t epnum)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;
  uint8_t ep_to_ch = 0;

  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    if (epnum == CDC_OUT_EP[i]) {
      ep_to_ch = i;
      break;
    }
  }

  hcdc = &CDC_ACM_Class_Data[ep_to_ch];

  /* Get the received data length */
  hcdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
     NAKed till the end of the application Xfer */

  ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)->Receive(ep_to_ch, hcdc->RxBuffer, &hcdc->RxLength);

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_EP0_RxReady
 *         Handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef * pdev)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;

  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    hcdc = &CDC_ACM_Class_Data[i];

    if (hcdc == NULL) { return (uint8_t) USBD_FAIL; }

    if ((pdev->pUserData_CDC_ACM != NULL) && (hcdc->CmdOpCode != 0xFFU)) {
      ((USBD_CDC_ACM_ItfTypeDef *) pdev->pUserData_CDC_ACM)
        ->Control(i, hcdc->CmdOpCode, (uint8_t *) hcdc->data[i], (uint16_t) hcdc->CmdLength);
      hcdc->CmdOpCode = 0xFFU;
    }
  }

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_GetFSCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t * USBD_CDC_GetFSCfgDesc(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_CDC_CfgFSDesc);

  return USBD_CDC_CfgFSDesc;
}

/**
 * @brief  USBD_CDC_GetHSCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t * USBD_CDC_GetHSCfgDesc(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_CDC_CfgHSDesc);

  return USBD_CDC_CfgHSDesc;
}

/**
 * @brief  USBD_CDC_GetOtherSpeedCfgDesc
 *         Return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t * USBD_CDC_GetOtherSpeedCfgDesc(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_CDC_CfgFSDesc);

  return USBD_CDC_CfgFSDesc;
}

/**
 * @brief  USBD_CDC_GetDeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t * USBD_CDC_GetDeviceQualifierDescriptor(uint16_t * length)
{
  *length = (uint16_t) sizeof(USBD_CDC_DeviceQualifierDesc);

  return USBD_CDC_DeviceQualifierDesc;
}

/**
 * @brief  USBD_CDC_RegisterInterface
 * @param  pdev: device instance
 * @param  fops: CD  Interface callback
 * @retval status
 */
uint8_t USBD_CDC_ACM_RegisterInterface(USBD_HandleTypeDef * pdev, USBD_CDC_ACM_ItfTypeDef * fops)
{
  if (fops == NULL) { return (uint8_t) USBD_FAIL; }

  pdev->pUserData_CDC_ACM = fops;

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_SetTxBuffer
 * @param  pdev: device instance
 * @param  pbuff: Tx Buffer
 * @retval status
 */
uint8_t USBD_CDC_SetTxBuffer(uint8_t ch, USBD_HandleTypeDef * pdev, uint8_t * pbuff, uint32_t length)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;

  hcdc = &CDC_ACM_Class_Data[ch];

  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_SetRxBuffer
 * @param  pdev: device instance
 * @param  pbuff: Rx Buffer
 * @retval status
 */
uint8_t USBD_CDC_SetRxBuffer(uint8_t ch, USBD_HandleTypeDef * pdev, uint8_t * pbuff)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;

  hcdc = &CDC_ACM_Class_Data[ch];

  hcdc->RxBuffer = pbuff;

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_CDC_TransmitPacket
 *         Transmit packet on IN endpoint
 * @param  pdev: device instance
 * @retval status
 */
uint8_t USBD_CDC_TransmitPacket(uint8_t ch, USBD_HandleTypeDef * pdev)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;
  USBD_StatusTypeDef ret = USBD_BUSY;

  hcdc = &CDC_ACM_Class_Data[ch];

  if (hcdc->TxState == 0U) {
    /* Tx Transfer in progress */
    hcdc->TxState = 1U;

    /* Update the packet total length */
    pdev->ep_in[CDC_IN_EP[ch] & 0xFU].total_length = hcdc->TxLength;

    /* Transmit next packet */
    (void) USBD_LL_Transmit(pdev, CDC_IN_EP[ch], hcdc->TxBuffer, hcdc->TxLength);

    ret = USBD_OK;
  }

  return (uint8_t) ret;
}

/**
 * @brief  USBD_CDC_ACM_ReceivePacket
 *         prepare OUT Endpoint for reception
 * @param  pdev: device instance
 * @retval status
 */
uint8_t USBD_CDC_ReceivePacket(uint8_t ch, USBD_HandleTypeDef * pdev)
{
  USBD_CDC_ACM_HandleTypeDef * hcdc = NULL;

  hcdc = &CDC_ACM_Class_Data[ch];

  if (pdev->dev_speed == USBD_SPEED_HIGH) {
    /* Prepare Out endpoint to receive next packet */
    (void) USBD_LL_PrepareReceive(pdev, CDC_OUT_EP[ch], hcdc->RxBuffer, CDC_DATA_HS_OUT_PACKET_SIZE);
  } else {
    /* Prepare Out endpoint to receive next packet */
    (void) USBD_LL_PrepareReceive(pdev, CDC_OUT_EP[ch], hcdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);
  }

  return (uint8_t) USBD_OK;
}

void USBD_Update_CDC_ACM_DESC(uint8_t * desc, uint8_t cmd_itf, uint8_t com_itf, uint8_t in_ep, uint8_t cmd_ep,
                              uint8_t out_ep, uint8_t str_idx)
{
  desc += 9;
  for (uint8_t i = 0; i < NUMBER_OF_CDC; i++) {
    desc[2] = cmd_itf;
    desc[16] = str_idx;
    desc[10] = cmd_itf;
    desc[26] = com_itf;
    desc[34] = cmd_itf;
    desc[35] = com_itf;
    desc[38] = cmd_ep;
    desc[45] = com_itf;
    desc[54] = out_ep;
    desc[61] = in_ep;

    desc += 66;
    CDC_IN_EP[i] = in_ep;
    CDC_OUT_EP[i] = out_ep;
    CDC_CMD_EP[i] = cmd_ep;
    CDC_CMD_ITF_NBR[i] = cmd_itf;
    CDC_COM_ITF_NBR[i] = com_itf;
    CDC_STR_DESC_IDX[i] = str_idx;

    in_ep += 2;
    cmd_ep = in_ep + 1;
    out_ep++;
    str_idx++;

    cmd_itf += 2;
    com_itf = cmd_itf + 1;
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
