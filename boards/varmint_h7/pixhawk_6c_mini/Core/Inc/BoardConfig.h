#ifndef PIXHAWK_6C_MINI_BOARDCONFIG_H
#define PIXHAWK_6C_MINI_BOARDCONFIG_H

#include "CommonConfig.h"

#define VCP_Transmit(buffer, length) CDC_Transmit(0, buffer, length)
#define VCP_HZ 400
#define ADC_CHANNELS 16

#define _USBD_USE_HS 0
#define _USBD_CDC_ACM_COUNT 1

#endif
