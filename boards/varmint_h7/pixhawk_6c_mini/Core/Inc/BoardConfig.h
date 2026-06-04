#ifndef PIXHAWK_6C_MINI_BOARDCONFIG_H
#define PIXHAWK_6C_MINI_BOARDCONFIG_H

#include "CommonConfig.h"

#define VCP_Transmit(buffer, length) CDC_Transmit(0, buffer, length)
#define VCP_HZ 400
#define ADC_CHANNELS 16

// FMU PWM output mapping.
// Channel numbers follow the board pinout labels:
// CH1-CH4 on TIM1, CH5-CH6 on TIM4, CH7-CH8 on TIM5.
#define PWM_CHANNELS (8)
#define PWM_TIMER_BLOCKS (3)

#define PWM_INIT_DEFINE \
{ \
  { (&htim1), 0, 50.0f, { 0, 1, 2, 3 } }, \
  { (&htim4), 0, 50.0f, { 255, 255, 4, 5 } }, \
  { (&htim5), 0, 50.0f, { 6, 7, 255, 255 } } \
}

#define _USBD_USE_HS 0
#define _USBD_CDC_ACM_COUNT 1

#endif
