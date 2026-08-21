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
