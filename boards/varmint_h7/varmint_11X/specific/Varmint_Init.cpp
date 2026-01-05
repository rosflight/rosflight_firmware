/**
 ******************************************************************************
 * File     : VarmintInit.cpp
 * Date     : June 3, 2024
 ******************************************************************************
 *
 * Copyright (c) 2024, AeroVironment, Inc.
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
#include "Varmint.h"

#include "BoardConfig.h"
#include "Spi.h"
#include "Time64.h"
#include "misc.h"

#include "usb_device.h"
#include "usbd_cdc_acm_if.h"

#include "main.h"
#include <ctime>

#include "Callbacks.h"
#include "Polling.h"
#include "sandbox.h"

#include "Mpu.h"

bool verbose = BOARD_STATUS_PRINT;

Time64 time64;
Polling polling;

////////////////////////////////////////////////////////////////////////////////////////
//
// Varmint Board
//
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @fn void init_board(void)
 * @brief Board Initialization
 *
 */

void Varmint::init_board(void)
{
  uint32_t init_status;

  //MPU_Config();
  MpuConfig();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  PeriphCommonClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  //     MX_TIM1_Init();					// PWM, initialized elsewhere
  //      MX_TIM3_Init();					// PWM, initialized elsewhere
  //      MX_TIM4_Init();					// PWM, initialized elsewhere
  //  MX_TIM5_Init(); 				// Time64, initialized elsewhere
  //  MX_TIM7_Init(); 				// Poll, initialized elsewhere
  //  MX_TIM8_Init(); 				// Time64, initialized elsewhere
  //  MX_TIM12_Init();  			// ADIS16500 Ext Clock, initialized elsewhere
  MX_TIM17_Init();            // Used for programmed time delays to interrupts.
  //  MX_USART1_UART_Init();	// uBlox, initialized elsewhere
  MX_USART2_UART_Init(); // Telem and Debug
                         //  MX_USART3_UART_Init();  // S.Bus, initialized elsewhere
                         //	 MX_UART6_Init();				// not used
                         //	 MX_UART7_Init();				// not used
                         //  MX_ADC1_Init();					// initialized elsewhere
                         //  MX_ADC3_Init();					// initialized elsewhere
                         //  MX_USB_DEVICE_Init();
  MX_FDCAN1_Init();      // not used
                         //  MX_SDMMC1_SD_Init();		// initialized elsewhere
  MX_RTC_Init();         // not used
  MX_CRC_Init();         // Used for SD Card data checksum
  MX_RNG_Init();         // not used

  MX_USB_OTG_FS_PCD_Init();

  MX_USB_DEVICE_Init();

  status_len_ = 0;

  // Startup Chained Timestamp Timers 1us rolls over in 8.9 years.
  init_status = time64.init(
    &htim5, // HTIM_LOW, (32-bit counter)
    TIM5, // HTIM_LOW_INSTANCE,
    &htim8, //HTIM_HIGH, (16-bit overflow counter)
    TIM8 // HTIM_HIGH_INSTANCE
  );

#define ASCII_ESC 27
  misc_printf("\n\n%c[H", ASCII_ESC); // home
  misc_printf("%c[2J", ASCII_ESC);    // clear screen

  misc_printf("\nTime64 Startup\n");
  misc_exit_status(init_status);
  status_list_[status_len_++] = &time64;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // IMU initialization

  #define IMU_HZ (400) // Hz

  misc_printf("\n\nADIS165xx (imu0) Initialization\n");

  init_status = imu0_.init(
    IMU_HZ,  // Hz, sample rate
    gpio("PB8"), // drdy
    &hspi4, // spi port
    gpio("PE4"), // chip select
    gpio("PE12"), // reset pin
    &htim12, TIM12, TIM_CHANNEL_1, // timer for external clock
    500, // us, period of external clock
    (const double[]){-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0} // rotation matrix into board coordinate system
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &imu0_;

  misc_printf("\n\nBMI088 (imu1) Initialization\n");
  #define BMI088_RANGE_A (3)   // 0,1,2,3 --> 3,6,12,24g for BMI088; 2 4 8 16g for BMI 085
  #define BMI088_RANGE_G (2)   // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
  init_status = imu1_.init(
    IMU_HZ,
    gpio("PA15"), // drdy pin (accel)
    &hspi1,
    gpio("PA4"), // accel chip select
    gpio("PD10"), // gyro chip select
    BMI088_RANGE_A, // 0,1,2,3 --> 3,6,12,24g for BMI088; 2 4 8 16g for BMI 085
    BMI088_RANGE_G, // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
    (const double[]){-1.0, 0.0, 0.0,   0.0, -1.0, 0.0,    0.0, 0.0, 1.0}
  );
  // note gyro drdy pin, PA1, exists but is not used
  misc_exit_status(init_status);
  status_list_[status_len_++] = &imu1_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Pitot/Baro initialization

  misc_printf("\n\nDLHRL20G (pitot) Initialization\n");                // I2C must already be initialized
  init_status = pitot_.init(
      100, // Hz
      gpio("PE9"), // data ready EXTI
      &hi2c1 // i2c port
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &pitot_;

  // Baro is DPS310
  #define DPS310_HZ (50) // up to 50 Hz.

  misc_printf("\n\nDPS310 (baro) Initialization\n");
  init_status = baro_.init(
    50, // Hz
    gpio("PD11"), // data ready (implies 3-wire mode)
    &hspi2, // SPI
    gpio("PE1") // chip select
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &baro_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Lidar initialization

  // Range Lidar Sensor on i2c2
  misc_printf("\n\nLIDARLITEV3 (range) Initialization\n");                // I2C must already be initialized
  init_status = range_.init(
      100, //LIDAR_HZ,
      &hi2c2 //LIDAR_I2C,
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &range_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Optical Flow initialization

  misc_printf("\n\nPMW3901 (oflow) Initialization\n");
  init_status = oflow_.init(
      IMU_HZ, // IMU Rate
      10, // Hz output rate
      100,  // us delay after IMU Trigger
      &hspi4, // spi4
      gpio("PE3"), // chip select
      gpio("PE10"), // data ready
      gpio("PH1"), // reset
      &htim17 // delay timer
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &oflow_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Mag initialization

  misc_printf("\n\nIIS2MDC (mag) Initialization\n");
  init_status = mag_.init(
    100, // Hz
    gpio("PE0"), // data ready
    &hspi2, // SPI
    gpio("PB12"), // chip select
    (const double[]){-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0} // rotation matrix
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &mag_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // GPS initialization

  misc_printf("\n\nUbx (gps) Initialization\n");
  init_status = gps_.init(
    10, // GPS_HZ,
    gpio("PA3"), // GPS_PPS_PORT, GPS_PPS_PIN,
    &huart1, // GPS_UART,
    USART1, // GPS_UART_INSTANCE,
    &hdma_usart1_rx, //GPS_UART_DMA,
    115200 //GPS_BAUD
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &gps_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RC/S.Bus initialization

  misc_printf("\n\nS.Bus (rc) Initialization\n");
  init_status = rc_.init(
      112, //RC_HZ,
      &huart3, // RC_UART,
      USART3, // RC_UART_INSTANCE,
      &hdma_usart3_rx, //RC_UART_DMA,
      100000 //RC_BAUD
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &rc_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ADC initialization

  init_status =
    adc_.init(
        10, // ADC_HZ,
        &hadc1, // ADC_ADC_EXTERNAL,
        ADC1, // ADC_ADC_INSTANCE_EXTERNAL,
        &hadc3, // ADC_ADC_INTERNAL,
        ADC3 // ADC_ADC_INSTANCE_INTERNAL
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &adc_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // COM initialization

  misc_printf("\n\nVcp (vcp) Initialization\n");
  init_status = vcp_.init(
      IMU_HZ // Hz for computing time between imu readings.
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &vcp_;

  misc_printf("\n\nTelem (telem) Initialization\n");
  init_status = telem_.init(
      IMU_HZ, // Hz, for computing time between imu readings.
      &huart2, // TELEM_UART,
      USART2, //TELEM_UART_INSTANCE,
      nullptr, // Polling, no DMA TELEM_UART_DMA (&hdma_usart2_rx)
      921600 // TELEM_BAUD
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &telem_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PWM initialization

  misc_printf("\n\nPWM (PWM) Initialization\n");

  // PWM_MAX_TIMERS = 3
  pwm_.timer_ =     std::array< PwmTimer, PWM_MAX_TIMERS>{{
    {&htim1, PWM_STANDARD, PWM_STD_RATE_HZ},
    {&htim3, PWM_STANDARD, PWM_STD_RATE_HZ},
    {&htim4, PWM_STANDARD, PWM_STD_RATE_HZ}
    // for an unused timer entry use: { nullptr, PWM_NONE, 0.0 }
  }};
  // PWM_MAX_CHANNELS = 12, max _exposed_ channels per timer is 4
  pwm_.channel_ =  std::array< PwmChannel, PWM_MAX_CHANNELS>{{
    { &htim1, TIM_CHANNEL_1 }, // Servo 1
    { &htim1, TIM_CHANNEL_2 }, // Servo 2
    { &htim1, TIM_CHANNEL_3 }, // Servo 3
    { &htim1, TIM_CHANNEL_4 }, // Servo 4
    { &htim4, TIM_CHANNEL_3 }, // Servo 5
    { &htim4, TIM_CHANNEL_3 }, // Servo 6
    { &htim4, TIM_CHANNEL_1 }, // Servo 7
    { &htim4, TIM_CHANNEL_4 }, // Servo 8
    { &htim3, TIM_CHANNEL_1 }, // Servo 9
    { &htim3, TIM_CHANNEL_2 } // Servo 10
    // for an unused channel use: { nullptr, TIM_CHANNEL_NONE}
  }};

  init_status = pwm_.init();
  misc_exit_status(init_status);
  status_list_[status_len_++] = &pwm_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Servo Power Supply initialization

  // Initialize the Digital Potentiometer to 8.16V, I2C must already be initialized
  misc_printf("\n\nServo Voltage Initialization\n");
  init_status = servoV_.init(
      &hi2c1, // MCP4017_I2C shared with pitot
      4.8 //SERVO_VOLTAGE
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &servoV_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // uSD Card initialization

  misc_printf("\n\nSDMMC Initialization\n");
  init_status = sd_.init( &hsd1, SDMMC1 );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &sd_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // LED initialization

  red_led_.init(gpio("PB2"), false);
  green_led_.init(gpio("PE15"), false);
  blue_led_.init(gpio("PE8"), false);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Review Status List

  misc_printf("\n\nStatus List:\n");
  for (uint32_t i = 0; i < status_len_; i++) {
    //status_list_[i]->print();
    if (status_list_[i]->initGood()) {
      misc_printf("\033[0;42m");
    } else {
      misc_printf("\033[0;41m");
    }
    misc_printf("%-16s Status: 0x%08X", status_list_[i]->name(), status_list_[i]->status());
    misc_printf("\033[0m\n");
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Interrupt initializations

  misc_printf("\n\nSet-up EXTI IRQ's\n");
  // Unused
  //HAL_NVIC_EnableIRQ(EXTI0_IRQn);     // IIS2MDC
  //HAL_NVIC_EnableIRQ(EXTI1_IRQn);     // Bosh IMU Gyro
  //HAL_NVIC_EnableIRQ(EXTI2_IRQn);     // J105_3_SYNC_IN
  //HAL_NVIC_EnableIRQ(EXTI4_IRQn);     // None

  HAL_NVIC_EnableIRQ(EXTI3_IRQn);     // uBlox GPS PPS
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);   // ADIS IMU (8) & Pitot (9) DRDY
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Bosh IMU Accel (15), J000 Jetson SYNC (13), DPS310 (11)
                                      // J105 DRDY - PMW3901 (10),

  __HAL_UART_ENABLE_IT(gps_.huart(), UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(rc_.huart(), UART_IT_IDLE);

  telem_.rxStart(); // Also enables its interrupts.

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // High Rate Timer initialization

  misc_printf("\n\nPolling Timer Initialization\n");
  init_status = polling.init(
    &htim7, //POLL_HTIM,
    TIM7, // POLL_HTIM_INSTANCE,
    TIM_CHANNEL_1, //POLL_TIM_CHANNEL
    100 // us Polling period 100us = 10kHz
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &polling;

#if SANDBOX
  misc_printf("\n\nStarting Sandbox\n");
  sandbox();
#else
  misc_printf("\n\nStarting Rosflight\n");
  verbose = false;
#endif
}
