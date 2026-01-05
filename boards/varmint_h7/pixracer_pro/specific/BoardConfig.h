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
#define USE_UART_TELEM false // 1 = use UART, 0 = use VCP for link to companion computer.

// UART used for printf's
#define MISC_HUART (&huart2)

/////////////////////////////////////////////////////////////////////////////////////////////
// USB MiddleWare
// See CommonConfig.h for more #defines

#define VCP_Transmit(buffer, length) CDC_Transmit(0, buffer, length)
//#define VCP_Transmit(buffer, length) CDC_Transmit_FS(buffer, length)
//#define VCP_Transmit(buffer, length) CDC_Transmit_HS(buffer, length)

#define _USBD_USE_HS false
extern PCD_HandleTypeDef hpcd_USB_OTG_FS; // USB FS (48 MB/s)
#define _USBD_CDC_ACM_COUNT 1
//
/////////////////////////////////////////////////////////////////////////////////////////////

// clang-format off
#define INTERFACE_LIST \
  Sbus rc_; \
  Ubx gps_; \
  Adc adc_; \
  Telem telem_; \
  Vcp vcp_; \
  Pwm pwm_; \
  Sd sd_; \
  Dps310 baro_; \
  Bmi088 imu0_; \
  Led red_led_; \
  Led green_led_; \
  Led blue_led_; \
  Ms4525 pitot_;  /* External */ \
  Ist8308 mag_; /* External */ \
  Lidarlitev3hp range_; /* External */ \
  Pmw3901 oflow_; /* External */ \

  /**/


// Onboard ADC's
#define ADC_HZ (10) // Maximum is 500 Hz.

#define ADC_ADC_EXTERNAL (&hadc1)
#define ADC_ADC_INSTANCE_EXTERNAL (ADC1)
#define ADC_EXT_DMA_RAM DMA_RAM
#define ADC_CHANNELS_EXT (4)

#define ADC_RSSI_V (0)          // INP 11
#define ADC_BATTERY_VOLTS (1)   // INP 14
#define ADC_BATTERY_CURRENT (2) // INP 15
#define ADC_5V0 (3)             // INP 18

#define ADC_ADC_INTERNAL (&hadc3)
#define ADC_ADC_INSTANCE_INTERNAL (ADC3)

#define ADC_INT_DMA_RAM BDMA_RAM // NOTE! ADC3 using BDMA so this needs to be in SRAM4
#define ADC_CHANNELS_INT (3)

#define ADC_STM_TEMPERATURE (0 + ADC_CHANNELS_EXT) // INP 18 (Internal)
#define ADC_STM_VBAT (1 + ADC_CHANNELS_EXT)        // INP 17 (Internal)
#define ADC_STM_VREFINT (2 + ADC_CHANNELS_EXT)     // INP 19 (Internal)

// NOTE! This lets us put all the config in one file
// clang-format off
#define ADC_CFG_CHANS_DEFINE \
{ \
  {ADC_REGULAR_RANK_1, ADC_CHANNEL_11, 1.000, 0.0},         /* ADC_RSSI_V */ \
  {ADC_REGULAR_RANK_2, ADC_CHANNEL_14, 12.62, 0.0},         /* ADC_BATTERY_VOLTS */ \
  {ADC_REGULAR_RANK_3, ADC_CHANNEL_15, 60.5, 0.0747},         /* ADC_BATTERY_CURRENT */ \
  {ADC_REGULAR_RANK_4, ADC_CHANNEL_18, 2.000, 0.0},         /* ADC_5V0 */ \
  {ADC_REGULAR_RANK_1, ADC_CHANNEL_TEMPSENSOR, 1.000, 0.0}, /* ADC_STM_TEMPERATURE */ \
  {ADC_REGULAR_RANK_2, ADC_CHANNEL_VBAT, 4.000, 0.0},       /* ADC_STM_VBAT */ \
  {ADC_REGULAR_RANK_3, ADC_CHANNEL_VREFINT, 1.0,0.0}        /* ADC_STM_VREFINT */ \
}
// clang-format on

#define ADC_CHANNELS (ADC_CHANNELS_EXT + ADC_CHANNELS_INT)

// Probes
// Probe PIN PG9
//#define PROBE1_HI HAL_GPIO_WritePin(PROBE1_GPIO_Port, PROBE1_Pin, GPIO_PIN_SET)
//#define PROBE1_LO HAL_GPIO_WritePin(PROBE1_GPIO_Port, PROBE1_Pin, GPIO_PIN_RESET)
//#define PROBE1_TOG HAL_GPIO_TogglePin(PROBE1_GPIO_Port, PROBE1_Pin)


#endif /* BOARDCONFIG_H_ */
