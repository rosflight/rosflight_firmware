/**
 ******************************************************************************
 * File     : BoardConfig.h
 * Date     : Sep 23, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023, AeroVironment, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2.Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3.Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 **/

#ifndef BOARDCONFIG_H_
#define BOARDCONFIG_H_

#include "CommonConfig.h"

#define SANDBOX false
#define BOARD_STATUS_PRINT false
#define USE_UART_TELEM 1 // 1 = use UART, 0 = use VCP for link to companion computer.

// UART used for printf's
#define MISC_HUART (&huart2)

/////////////////////////////////////////////////////////////////////////////////////////////
// USB MiddleWare
// See CommonConfig.h for more #defines

#define VCP_Transmit(buffer, length) CDC_Transmit(0, buffer, length)

//#define _USBD_USE_HS false
extern PCD_HandleTypeDef hpcd_USB_OTG_FS; // USB FS (48 MB/s)
#define _USBD_CDC_ACM_COUNT 1
//
/////////////////////////////////////////////////////////////////////////////////////////////

//clang-format off

// This list is used in Varmint.h
#define INTERFACE_LIST  \
  Sbus rc_; \
  Ubx gps_; \
  Adc<6,0,3> adc_; /* channels in adc1, adc2, & adc3 */ \
  Telem telem_; \
  Vcp vcp_; \
  Pwm pwm_; \
  Sd sd_; \
  Dps310 baro_; \
  Adis165xx imu0_; \
  Bmi088 imu1_; \
  Iis2mdc mag_; \
  DlhrL20G pitot_; \
  Lidarlitev3hp range_; /* External I2C Lidar Range Sensor */ \
  Pmw3901 oflow_; /* External SPI Optical Flow Sensor */ \
  Led red_led_; \
  Led green_led_; \
  Led blue_led_;

//clang-format on

// Onboard ADC's

//#define ADC_ADC_EXTERNAL (&hadc1)
//#define ADC_ADC_INSTANCE_EXTERNAL (ADC1)

//#define ADC_EXT_DMA_RAM DMA_RAM

// only the indices for ADC_BATTERY_* are used by the board.
#define ADC_CC_3V3 (0)          // INP 4
#define ADC_SERVO_VOLTS (1)     // INP 7
#define ADC_12V (2)             // INP 8
#define ADC_5V0 (3)             // INP 10
#define ADC_BATTERY_CURRENT (4) // INP 11
#define ADC_BATTERY_VOLTS (5)   // INP 16
#define ADC_CHANNELS_EXT (6)

//#define ADC_ADC_INTERNAL (&hadc3)
//#define ADC_ADC_INSTANCE_INTERNAL (ADC3)

//#define ADC_INT_DMA_RAM BDMA_RAM // NOTE! ADC3 using BDMA so this needs to be in SRAM4

#define ADC_STM_TEMPERATURE (0 + ADC_CHANNELS_EXT) // INP 18 (Internal)
#define ADC_STM_VBAT (1 + ADC_CHANNELS_EXT)        // INP 17 (Internal)
#define ADC_STM_VREFINT (2 + ADC_CHANNELS_EXT)     // INP 19 (Internal)
#define ADC_CHANNELS_INT (3)

// NOTE! This lets us put all the config in one file
// clang-format off
#define ADC_CFG_CHANS_DEFINE \
{ \
  {ADC_REGULAR_RANK_1, ADC_CHANNEL_4, 1.100, 0.0},          /* ADC_CC_3V3 */                   \
  {ADC_REGULAR_RANK_2, ADC_CHANNEL_7, 2.690, 0.0},          /* ADC_SERVO_VOLTS */              \
  {ADC_REGULAR_RANK_3, ADC_CHANNEL_8, 4.010, 0.0},          /* ADC_12V */                      \
  {ADC_REGULAR_RANK_4, ADC_CHANNEL_10, 1.680, 0.0},         /* ADC_5V0 */                      \
  {ADC_REGULAR_RANK_5, ADC_CHANNEL_11, 10.000, 0.0},        /* ADC_BATTERY_CURRENT */          \
  {ADC_REGULAR_RANK_6, ADC_CHANNEL_16, 11.215, 0.0},        /* ADC_BATTERY_VOLTS */            \
\
  {ADC_REGULAR_RANK_1, ADC_CHANNEL_TEMPSENSOR, 1.000, 0.0}, /* ADC_STM_TEMPERATURE */          \
  {ADC_REGULAR_RANK_2, ADC_CHANNEL_VBAT, 4.000, 0.0},       /* ADC_STM_VBAT */                 \
  {ADC_REGULAR_RANK_3, ADC_CHANNEL_VREFINT, 1.000, 0.0}     /* ADC_STM_VREFINT */              \
}
// clang-format on

#define ADC_CHANNELS (ADC_CHANNELS_EXT + ADC_CHANNELS_INT)


//// PB15
//#define PROBE3_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
//#define PROBE3_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
//#define PROBE3_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15)
//
//// Real Board - PB1 J105 pin 23/25 Sync Bus (CAN XCVR)
//#define PROBE4_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
//#define PROBE4_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
//#define PROBE4_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)

#endif /* BOARDCONFIG_H_ */
