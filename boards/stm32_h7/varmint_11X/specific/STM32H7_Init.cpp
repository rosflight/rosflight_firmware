/**
 ******************************************************************************
 * File     : STM32H7_Init.cpp
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
#include <stm32_h7.hpp>

#include <BoardConfig.h>
#include <Spi.h>
#include <Time64.h>
#include <misc.h>

#include <usb_device.h>
#include <usbd_cdc_acm_if.h>

#include <ctime>
#include <main.h>

#include <Callbacks.h>
#include <Polling.h>
#include <sandbox.h>

#include <Mpu.h>

bool verbose = BOARD_STATUS_PRINT;

Time64 time64;

////////////////////////////////////////////////////////////////////////////////////////
//
// STM32H7 Board
//
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @fn void init_board(void)
 * @brief Board Initialization
 *
 */

void STM32H7Board::init_board(void)
{
// clang-format off

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

#if _USBD_USE_HS // USB HS (480MB/s
  MX_USB_OTG_HS_PCD_Init();
#else // USB FS (48 MB/s)
  MX_USB_OTG_FS_PCD_Init();
#endif

  MX_USB_DEVICE_Init();

  status_len_ = 0;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Timestamp Timers 1us rolls over in 8.9 years.

  init_status = time64.init(
    &htim5, TIM5, // 32-bit counter
    &htim8, TIM8  // 16-bit overflow counter
  );
  
  #define ASCII_ESC 27
  misc_printf("\n\n%c[H", ASCII_ESC); // home
  misc_printf("%c[2J", ASCII_ESC);    // clear screen

  misc_printf("\nTime64 Startup\n");
  misc_exit_status(init_status);
  status_list_[status_len_++] = &time64;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Callbacks initialization

  callbacks().clear_all();

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // IMU initialization

  misc_printf("\n\nADIS165xx (imu0) Initialization\n");
  init_status = imu0_.init(
    400, // sample rate, Hz
    ADIS165XX_DRDY_GPIO_Port, ADIS165XX_DRDY_Pin, // DRDY Port
    &hspi4, ADIS165XX_CSn_GPIO_Port, ADIS165XX_CSn_Pin,     // SPI
    ADIS165XX_RESET_GPIO_Port, ADIS165XX_RESET_Pin,         // Reset Pin
    &htim12, TIM12, TIM_CHANNEL_1, 500, // Timer, 500us period
    (const double[]){-1.0, 0.0, 0.0,   0.0, -1.0, 0.0,    0.0, 0.0, 1.0} // rotation into board coordinate system.
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &imu0_;
  if (init_status == DRIVER_OK) { imu0_.register_callbacks(*this); }

  misc_printf("\n\nBMI088 (imu1) Initialization\n");
  init_status = imu1_.init(
    400, // Sample Rat, Hz, 400, 1000, 2000 are the only options
	BMI088_ACCEL_DRDY_GPIO_Port, BMI088_ACCEL_DRDY_Pin, // DRDY
	&hspi1, // SPI
    BMI088_ACCEL_CSn_GPIO_Port, BMI088_ACCEL_CSn_Pin, // Accel Chip Select
	BMI088_GYRO_CSn_GPIO_Port, BMI088_GYRO_CSn_Pin,   // Gyro Chip Select
	3, // 0,1,2,3 --> 3,6,12,24g for BMI088; 2 4 8 16g for BMI 085
	2, // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
	(const double[]){-1.0, 0.0, 0.0,   0.0, -1.0, 0.0,    0.0, 0.0, 1.0} // Rotation
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &imu1_;
  if (init_status == DRIVER_OK) { imu1_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Pitot/Baro initialization

  misc_printf("\n\nDLHRL20G (pitot) Initialization\n"); // I2C must already be initialized
  init_status = pitot_.init(
    100, // Sample Rate, Hz
	PITOT_DRDY_GPIO_Port, PITOT_DRDY_Pin, // Driver
	&hi2c1, DLHRL20G_I2C_ADDRESS // I2C
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &pitot_;
  if (init_status == DRIVER_OK) { pitot_.register_callbacks(*this); }

  misc_printf("\n\nDPS310 (baro) Initialization\n");
  init_status = baro_.init(
    50, // Sample Rate, Hz, up to 50 Hz.
	DPS310_DRDY_GPIO_Port, DPS310_DRDY_Pin, // DRDY
	&hspi2, // SPI
	DPS310_CSn_GPIO_Port, DPS310_CSn_Pin   // Chip Select
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &baro_;
  if (init_status == DRIVER_OK) { baro_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Mag initialization

  misc_printf("\n\nIIS2MDC (mag) Initialization\n");
  init_status = mag_.init(
    100, // Sample Rate, Hz.
	IIS2MDC_DRDY_GPIO_Port, IIS2MDC_DRDY_Pin, // Driver
	&hspi2, // SPI
	IIS2MDC_CSn_GPIO_Port, IIS2MDC_CSn_Pin,  // SPI
	(const double[]){-1.0, 0.0, 0.0,   0.0, 1.0, 0.0,    0.0, 0.0, -1.0}
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &mag_;
 if (init_status == DRIVER_OK) { mag_.register_callbacks(*this, 1); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // GPS initialization

  misc_printf("\n\nUbx (gps) Initialization\n");
  init_status = gps_.init(
    10, // Sample Rate, Hz
	  GPS_1PPS_GPIO_Port, GPS_1PPS_Pin, // PPS EXTI
	  &huart1, USART1,
	  &hdma_usart1_rx, // UART DMA
	115200 // Baud
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &gps_;
  if (init_status == DRIVER_OK) { gps_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RC/S.Bus initialization

  misc_printf("\n\nS.Bus (rc) Initialization\n");
  init_status = rc_.init(
    112,  // Frame Rate, 1000/9ms = 111.1Hz, 112 is rounded up
	  &huart3, USART3, // UART
	  &hdma_usart3_rx, // UART DMA
	  100000 // Baud,
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &rc_;
  if (init_status == DRIVER_OK) { rc_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ADC initialization

  misc_printf("\n\nAdc (adc) Initialization\n");

  #define ADC_CHANNELS_EXT (6)
  #define ADC_BATTERY_VOLTS (0)   // INP 16
  #define ADC_BATTERY_CURRENT (1) // INP 11
  #define ADC_12V (2)             // INP 8
  #define ADC_5V0 (3)             // INP 10
  #define ADC_SERVO_VOLTS (4)     // INP 7
  #define ADC_CC_3V3 (5)          // INP 4

  #define ADC_CHANNELS_INT (3)
  #define ADC_STM_TEMPERATURE (6) // INP 18 (Internal)
  #define ADC_STM_VBAT (7)        // INP 17 (Internal)
  #define ADC_STM_VREFINT (8)     // INP 19 (Internal)

  #define ADC_CHANNELS (ADC_CHANNELS_EXT + ADC_CHANNELS_INT)

  static const AdcChannelCfg board_adc_cfg[ADC_CHANNELS] = {
	  {ADC_REGULAR_RANK_1, ADC_CHANNEL_16, 11.215, 0.0},
	  {ADC_REGULAR_RANK_2, ADC_CHANNEL_11, 10.000, 0.0},
    {ADC_REGULAR_RANK_3, ADC_CHANNEL_8, 4.010, 0.0},
    {ADC_REGULAR_RANK_4, ADC_CHANNEL_10, 1.680, 0.0},
    {ADC_REGULAR_RANK_5, ADC_CHANNEL_7, 2.690, 0.0},
	  {ADC_REGULAR_RANK_6, ADC_CHANNEL_4, 1.100, 0.0},
    {ADC_REGULAR_RANK_1, ADC_CHANNEL_TEMPSENSOR, 1.000, 0.0},
    {ADC_REGULAR_RANK_2, ADC_CHANNEL_VBAT, 4.000, 0.0},
    {ADC_REGULAR_RANK_3, ADC_CHANNEL_VREFINT, 1.000, 0.0},
  };
  // Names that go with the channels
  static const char* adc_names[2*ADC_CHANNELS] = {
    "V_BATT", "V",
	  "I_BATT", "A",
    "12V", "V",
    "5V0", "V",
    "V_SERVO", "V",
    "CC_3V3", "V",
    "TEMP", "C",
    "V_RTC", "V",
    "V_REF", "V",   
  };

  static const AdcStructure board_adc_init = {
    ADC_CHANNELS_EXT, 
    ADC_CHANNELS_INT, 
    ADC_BATTERY_VOLTS, 
    ADC_BATTERY_CURRENT, 
    ADC_STM_TEMPERATURE, 
    ADC_STM_VBAT, 
    ADC_STM_VREFINT, 
    -1, 
    board_adc_cfg,
    adc_names
  };
  init_status = adc_.init(
    10, // Sample Rate, Hz
	&hadc1, ADC1, // "External"
	&hadc3, ADC3, // "Internal" has the on chip sensors
	&board_adc_init
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &adc_;
  if (init_status == DRIVER_OK) { adc_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // COM initialization

  misc_printf("\n\nVcp (vcp) Initialization\n");
  init_status = vcp_.init(
    EPOCH_HZ // Highest Sensor Sample Rate
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &vcp_;
   if (init_status == DRIVER_OK) { vcp_.register_callbacks(*this); }

  misc_printf("\n\nTelem (telem) Initialization\n");
  init_status = telem_.init(
    EPOCH_HZ, // Highest Sensor Sample Rate
	&huart2, USART2,
	0, // (&hdma_usart2_rx), 0 = no dma, using isr
	921600
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &telem_;
  if (init_status == DRIVER_OK) { telem_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PWM initialization

  misc_printf("\n\nPWM (PWM) Initialization\n");
// Channel order based on hardware pinout naming
//	TIMER 1 TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
//	TIMER 4 TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_4,
//	TIMER 3 TIM_CHANNEL_1, TIM_CHANNEL_2
  static const PwmBlockStructure board_pwm_init[] = 
  { \
    {(&htim1), PWM_STANDARD, PWM_STD_RATE_HZ, {0, 1, 2, 3}}, \
    {(&htim4), PWM_STANDARD, PWM_STD_RATE_HZ, {6, 5, 4, 7}}, \
    {(&htim3), PWM_STANDARD, PWM_STD_RATE_HZ, { 8, 9, 255, 255 }} \
  };
  init_status = pwm_.init(
    10, // Number of PWM output channels on the board
    3, // Number of Timer blocks
    board_pwm_init
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &pwm_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // uSD Card initialization
  misc_printf("\n\nSDMMC Initialization\n");
  init_status = sd_.init(&hsd1, SDMMC1);
  misc_exit_status(init_status);
  status_list_[status_len_++] = &sd_;
  if (init_status == DRIVER_OK) { sd_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Servo Power Supply initialization

  misc_printf("\n\nServo Voltage Initialization\n");
  init_status = servoV_.init(
    &hi2c1, //I2C
    MCP4017_I2C_ADDRESS, //
    4.8 // Servo voltage, V
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &servoV_;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Interrupt initializations

  misc_printf("\n\nSet-up EXTI IRQ's\n");

  HAL_NVIC_EnableIRQ(EXTI3_IRQn);     // uBlox GPS PPS
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);   // ADIS IMU DRDY
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Bosh IMU DRDY

  __HAL_UART_ENABLE_IT(gps_.huart(), UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(rc_.huart(), UART_IT_IDLE);

  telem_.rxStart(); // Also enables its interrupts.

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // High Rate Polling Timer initialization

  misc_printf("\n\nPolling Timer Initialization\n");
  init_status = polling_timer_.init(
    &htim7, TIM7, TIM_CHANNEL_1, // POLL Timer,
    100 // Poll Period, us. (10kHz)
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &polling_timer_;

  RED_LO;
  GRN_LO;
  BLU_LO;

#if SANDBOX
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Review Status List

  // misc_printf("\n\nStatus List:\n");
  // for (uint32_t i = 0; i < status_len_; i++) {
  //   //status_list_[i]->print();
  //   if (status_list_[i]->initGood()) {
  //     misc_printf("\033[0;42m");
  //   } else {
  //     misc_printf("\033[0;41m");
  //   }
  //   misc_printf("%-16s Status: 0x%08X", status_list_[i]->name(), status_list_[i]->status());
  //   misc_printf("\033[0m\n");
  // }
  misc_printf("\n\nStarting Sandbox\n");
  sandbox();
#else
  misc_printf("\n\nStarting Rosflight\n");
  verbose = false;
#endif
}
