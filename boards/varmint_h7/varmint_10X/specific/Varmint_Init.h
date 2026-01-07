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

#ifndef VARMINT_INIT_H_
#define VARMINT_INIT_H_

#include "CommonConfig.h"

#define SANDBOX // define to use
#define BOARD_STATUS_PRINT // define to use

#define USE_UART_TELEM 0 // 1 = use UART, 0 = use VCP for link to companion computer.

// UART used for printf's
//#define MISC_HUART (&huart2)

/////////////////////////////////////////////////////////////////////////////////////////////
// USB MiddleWare
// See CommonConfig.h for more #defines

// Override these if you are using HS vs FS
//#define _USBD_USE_HS false
//#define _USBD_USE_HS true // true/false (for FS, use "false")
//#define _USBD_CDC_ACM_COUNT 1

//
/////////////////////////////////////////////////////////////////////////////////////////////

//clang-format off

// This list is used in Varmint.h
#define INTERFACE_LIST  \
  Sbus rc_; \
  Ubx gps_; \
  Adc adc_; /* channels in adc1, & adc3 */ \
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


//// PB15
//#define PROBE3_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
//#define PROBE3_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
//#define PROBE3_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15)
//
//// Real Board - PB1 J105 pin 23/25 Sync Bus (CAN XCVR)
//#define PROBE4_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
//#define PROBE4_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
//#define PROBE4_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)

#endif /* VARMINT_INIT_H_ */
