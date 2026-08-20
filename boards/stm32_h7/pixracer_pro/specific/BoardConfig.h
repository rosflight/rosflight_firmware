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
#define BOARD_STATUS_PRINT (false|SANDBOX)
#define USE_TELEM 0 // 1 = use UART, 0 = use VCP for link to companion computer.

// UART used for printf's
#define MISC_HUART (&huart2)

/////////////////////////////////////////////////////////////////////////////////////////////
// USB MiddleWare
// See CommonConfig.h for more #defines

#define VCP_Transmit(buffer, length) CDC_Transmit(0, buffer, length)

#define _USBD_USE_HS false
extern PCD_HandleTypeDef hpcd_USB_OTG_FS; // USB FS (48 MB/s)
#define _USBD_CDC_ACM_COUNT 1
//
/////////////////////////////////////////////////////////////////////////////////////////////

// clang-format off
#define INTERFACE_LIST \
  Sbus rc_;			/* All */ \
  Ubx gps_;			/* All */ \
  Adc adc_;			/* All */ \
  Telem telem_;		/* All */ \
  Vcp vcp_;			/* All */ \
  Pwm pwm_;			/* All */ \
  Sd sd_; 			/* All */ \
  Dps310 baro_;		/* All */ \
  Bmi088 imu0_; 		/* PixRacer Pro */\
  Ms4525 pitot_;   	/* PixRacer Pro */ \
  Ist8308 mag_;	 	/* PixRacer Pro */ \
  /**/
// clang-format on

// 48-bit us counter.
// Prefer to have the 32-bit counter on the low order bytes:
#define HTIM_LOW (&htim5) // 32-bit counter
#define HTIM_LOW_INSTANCE (TIM5)
#define HTIM_HIGH (&htim12) // 16-bit overflow counter
#define HTIM_HIGH_INSTANCE (TIM12)

#define POLL_HTIM (&htim7) // High rate periodic interrupt timer (PITR)
#define POLL_TIM_CHANNEL TIM_CHANNEL_1
#define POLL_HTIM_INSTANCE (TIM7)
#define POLLING_PERIOD_US (100)                       // 100us, 10kHz
#define POLLING_FREQ_HZ (1000000 / POLLING_PERIOD_US) // 10000 Hz

///////////////////////////////////////////////////////////////////////////////////////////////////
// Pwm's

// Keep this for if we activate DSHOT
//#define DSHOT_ESC_MIN		(48)
//#define DSHOT_ESC_MAX		(2047)
//
//#define PWM_DSHOT_RATE_HZ 	(300000.0) // baud rate
//#define PWM_MKS_RATE_HZ 	(333.0)
//#define PWM_STD_RATE_HZ 	(50.0)
//typedef enum : uint8_t
//{
//	PWM_STANDARD,
//	PWM_DSHOT
//} pwm_type;

#define PWM_CHANNELS (8) // Number of PWM output channels on the board
#define PWM_TIMER_BLOCKS 3

// Arrays are the mapping of CH1-4 to the 10 PWM Channels Index
// clang-format off
#define PWM_INIT_DEFINE \
{ \
  { (&htim1), PWM_STANDARD, PWM_STD_RATE_HZ, { 3,   2,  1,   0}}, \
  { (&htim4), PWM_STANDARD, PWM_STD_RATE_HZ, { 255, 4,  5, 255}}, \
  { (&htim8), PWM_STANDARD, PWM_STD_RATE_HZ, { 6,  7, 255, 255}}  \
}
// clang-format on
// Channel order based on hardware pinout naming
//	TIMER 1 TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1
//	TIMER 4 TIM_CHANNEL_2, TIM_CHANNEL_3
//	TIMER 8 TIM_CHANNEL_1, TIM_CHANNEL_2

///////////////////////////////////////////////////////////////////////////////////////////////////
// Onboard ADC's

#define ADC_EXT_DMA_RAM DMA_RAM
#define ADC_CHANNELS_EXT (4)

#define ADC_RSSI_V (0)          // INP 11
#define ADC_BATTERY_VOLTS (1)   // INP 14
#define ADC_BATTERY_CURRENT (2) // INP 15
#define ADC_5V0 (3)             // INP 18

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

// Red LED
// PB11
#define RED_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define RED_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
#define RED_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11)
// Green LED
// PB1
#define GRN_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define GRN_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define GRN_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)
// Blue LED
// PB3
#define BLU_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define BLU_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define BLU_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3)

// Probes
#if 0
  #define PROBE1_HI HAL_GPIO_WritePin(PROBE1_GPIO_Port, PROBE1_Pin, GPIO_PIN_SET)
  #define PROBE1_LO HAL_GPIO_WritePin(PROBE1_GPIO_Port, PROBE1_Pin, GPIO_PIN_RESET)
  #define PROBE1_TOG HAL_GPIO_TogglePin(PROBE1_GPIO_Port, PROBE1_Pin)

  #define PROBE2_HI HAL_GPIO_WritePin(PROBE2_GPIO_Port, PROBE2_Pin, GPIO_PIN_SET)
  #define PROBE2_LO HAL_GPIO_WritePin(PROBE2_GPIO_Port, PROBE2_Pin, GPIO_PIN_RESET)
  #define PROBE2_TOG HAL_GPIO_TogglePin(PROBE2_GPIO_Port, PROBE2_Pin)

  #define PROBE3_HI HAL_GPIO_WritePin(PROBE3_GPIO_Port, PROBE3_Pin, GPIO_PIN_SET)
  #define PROBE3_LO HAL_GPIO_WritePin(PROBE3_GPIO_Port, PROBE3_Pin, GPIO_PIN_RESET)
  #define PROBE3_TOG HAL_GPIO_TogglePin(PROBE3_GPIO_Port, PROBE3_Pin)

#endif

#endif /* BOARDCONFIG_H_ */
