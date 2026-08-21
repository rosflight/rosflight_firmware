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
#include "stm32_h7.hpp"

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

namespace
{
class Bmi088GyroBridgeSet
{
public:
  bool isMy(uint16_t exti_pin) { return exti_pin == BMI088_INT4_GYRO_Pin; }

  void extiCallback()
  {
    HAL_GPIO_WritePin(BMI088_INT2_ACCEL_GPIO_Port, BMI088_INT2_ACCEL_Pin, GPIO_PIN_SET);
  }
};

class Bmi088AccelBridgeClear
{
public:
  bool isMy(uint16_t exti_pin) { return exti_pin == BMI088_INT1_ACCEL_Pin; }

  void extiCallback()
  {
    HAL_GPIO_WritePin(BMI088_INT2_ACCEL_GPIO_Port, BMI088_INT2_ACCEL_Pin, GPIO_PIN_RESET);
  }
};

Bmi088GyroBridgeSet bmi088_gyro_bridge_set;
Bmi088AccelBridgeClear bmi088_accel_bridge_clear;
} // namespace

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
  //	MX_SDMMC1_SD_Init(); // initialized elsewhere
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  //  MX_SPI6_Init(); // NOt using

  //	MX_TIM1_Init();  // PWM, initialized elsewhere
  //	MX_TIM2_Init();	 // PWM (Buzzer) not used as such
  //	MX_TIM3_Init();  // RC PPM input, not used as such
  //	MX_TIM4_Init();  // PWM, initialized elsewhere
  //	MX_TIM8_Init();  // PWM, initialized elsewhere
  //  MX_TIM7_Init();  // Poll, initialized elsewhere
  //	MX_TIM5_Init();  // Time64, initialized elsewhere
  //	MX_TIM15_Init(); // Time64, initialized elsewhere

  //	MX_UART4_Init(); // GPS, initialized elsewhere
  //	MX_UART7_Init(); // Serial 5&6 connector, not used
  //	MX_UART8_Init(); // Telem "FRSKY" connector
  //	MX_USART1_UART_Init(); // Serial 5&6 connector, not used
  MX_USART2_UART_Init(); // Telem/Serial 1
  //	MX_USART3_UART_Init(); // Serial 2 connector, not used
  //	MX_USART6_UART_Init(); // RC UART, initialized elsewhere

  //	MX_ADC3_Init(); // initialized elsewhere
  //	MX_ADC1_Init(); // initialized elsewhere

  MX_FDCAN1_Init(); // not used yet
  MX_FDCAN2_Init(); // not used yet

  MX_CRC_Init(); // Used for SD Card data checksum
  MX_RNG_Init(); // not used
  MX_RTC_Init(); // not used

#if _USBD_USE_HS // USB HS (480MB/s
  MX_USB_OTG_HS_PCD_Init();
#else // USB FS (48 MB/s)
  MX_USB_OTG_FS_PCD_Init();
#endif
  MX_USB_DEVICE_Init();

  status_len_ = 0;

  //// Startup Chained Timestamp Timers 1us rolls over in 8.9 years.
  //misc_printf("\nStarted Timestamp Timer\n");
  init_status = time64.init(HTIM_LOW, HTIM_LOW_INSTANCE, HTIM_HIGH, HTIM_HIGH_INSTANCE);

  // misc_printf uses the timer, so can't be used before it's initialized.

#define ASCII_ESC 27
  misc_printf("\n\n%c[H", ASCII_ESC); // home
  misc_printf("%c[2J", ASCII_ESC);    // clear screen

  misc_printf("\nTime64 Startup\n");
  misc_exit_status(init_status);
  status_list_[status_len_++] = &time64;
  clear_poll_clients();
  clear_exti_clients();
  clear_spi_clients();
  clear_i2c_clients();
  clear_adc_clients();
  clear_cdc_clients();
  clear_sd_clients();
  clear_uart_rxcplt_clients();
  clear_uart_rxisr_clients();
  clear_uart_txcplt_clients();

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // IMU initialization

  HAL_NVIC_DisableIRQ(BMI088_INT4_GYRO_EXTI_IRQn);  // EXTI4_IRQn Gyro DRDY Feedback
  HAL_NVIC_DisableIRQ(BMI088_INT1_ACCEL_EXTI_IRQn); // EXTI1_IRQn ACCEL DRDY

  misc_printf("\n\nBMI088 (imu1) Initialization\n");
  init_status = imu0_.init(
    400, // BMI088_HZ,
    BMI088_INT1_ACCEL_GPIO_Port, BMI088_INT1_ACCEL_Pin, // DRDY for Accel
    &hspi5,	// SPI
    BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin,// Accel Chip Select pin
    BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin,  // Gyro Chip Select pin
    3, // 0,1,2,3 --> 3,6,12,24g for BMI088; 2 4 8 16g for BMI 085
    2, // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
    (const double[]){ -1.0, 0.0, 0.0,   0.0, -1.0, 0.0,    0.0, 0.0, 1.0} //BMI088_ROTATION
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &imu0_;
  if (init_status == DRIVER_OK) { register_exti_client(&bmi088_gyro_bridge_set); }
  if (init_status == DRIVER_OK) { register_exti_client(&bmi088_accel_bridge_clear); }
  if (init_status == DRIVER_OK) { imu0_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Pitot/Baro initialization

  misc_printf("\n\nMS4525 (Pitot) Initialization\n"); // I2C must already be initialized
  init_status = pitot_.init(
    100, // Hz, rate
	&hi2c1,
	MS4525_I2C_ADDRESS
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &pitot_;
  if (init_status == DRIVER_OK) { pitot_.register_callbacks(*this, -5); }

  misc_printf("\n\nDPS310 (baro) Initialization\n");
  init_status = baro_.init(
    50, // Sample Rate Hz
	&hspi2,
    DPS310_CSn_GPIO_Port, DPS310_CSn_Pin // Chip Select
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &baro_;
  if (init_status == DRIVER_OK) { baro_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Mag initialization

  misc_printf("\n\nIST3808 (mag) Initialization\n");
  init_status = mag_.init(
    100, // Sample Rate (Hz)
    &hi2c1, // I2C1
    0X0C, // I2C Address
    (const double[]){1.0, 0.0, 0.0,   0.0, 1.0, 0.0,    0.0, 0.0, 1.0}
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &mag_;
  if (init_status == DRIVER_OK) { mag_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // GPS initialization

  misc_printf("\n\nUbx (gps) Initialization\n");
  init_status = gps_.init(
    10, // Sample Rate, Hz
	GPS_PPS_GPIO_Port, GPS_PPS_Pin, // PPS Pin
	&huart4, UART4, // UART
	&hdma_uart4_rx, // UART DMA
	115200 // BAUD
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &gps_;
  if (init_status == DRIVER_OK) { gps_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RC/S.Bus initialization

  misc_printf("\n\nS.Bus (rc) Initialization\n");
  init_status = rc_.init(
	112, // Frame Rate (approximate 1000/9ms = 111.1Hz, 112 is rounded up)
	&huart6, USART6,
	&hdma_usart6_rx,
	100000
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &rc_;
  if (init_status == DRIVER_OK) { rc_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // ADC initialization

  misc_printf("\n\nAdc (adc) Initialization\n");
  #define ADC_CHANNELS_EXT (4)
  #define ADC_RSSI_V (0)          // INP 11
  #define ADC_BATTERY_VOLTS (1)   // INP 14
  #define ADC_BATTERY_CURRENT (2) // INP 15
  #define ADC_5V0 (3)             // INP 18

  #define ADC_CHANNELS_INT (3)
  #define ADC_STM_TEMPERATURE (0 + ADC_CHANNELS_EXT) // INP 18 (Internal)
  #define ADC_STM_VBAT (1 + ADC_CHANNELS_EXT)        // INP 17 (Internal)
  #define ADC_STM_VREFINT (2 + ADC_CHANNELS_EXT)     // INP 19 (Internal)
  
  #define ADC_CHANNELS (ADC_CHANNELS_EXT + ADC_CHANNELS_INT)

  static const AdcChannelCfg board_adc_cfg[ADC_CHANNELS] = {
    {ADC_REGULAR_RANK_1, ADC_CHANNEL_11, 1.000, 0.0},
    {ADC_REGULAR_RANK_2, ADC_CHANNEL_14, 12.62, 0.0},
    {ADC_REGULAR_RANK_3, ADC_CHANNEL_15, 60.5, 0.0747},
    {ADC_REGULAR_RANK_4, ADC_CHANNEL_18, 2.000, 0.0},
    {ADC_REGULAR_RANK_1, ADC_CHANNEL_TEMPSENSOR, 1.000, 0.0},
    {ADC_REGULAR_RANK_2, ADC_CHANNEL_VBAT, 4.000, 0.0},
    {ADC_REGULAR_RANK_3, ADC_CHANNEL_VREFINT, 1.0, 0.0},
  };

  static const char* adc_names[2*ADC_CHANNELS] = {
    "V_RSSI", "V",
    "V_BATT", "V",
    "I_BATT", "A",
    "5V0", "V",
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
	  0, // &hdma_usart2_rx, 0 = none, we are using ISR
	  921600
  );
  misc_exit_status(init_status);
  status_list_[status_len_++] = &telem_;
  if (init_status == DRIVER_OK) { telem_.register_callbacks(*this); }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PWM initialization

  misc_printf("\n\nPWM (PWM) Initialization\n");
// Channel order based on hardware pinout naming
//	TIMER 1 TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1
//	TIMER 4 TIM_CHANNEL_2, TIM_CHANNEL_3
//	TIMER 8 TIM_CHANNEL_1, TIM_CHANNEL_2
  static const PwmBlockStructure board_pwm_init[] = 
  { \
    { (&htim1), PWM_STANDARD, PWM_STD_RATE_HZ, { 3,   2,  1,   0}}, \
    { (&htim4), PWM_STANDARD, PWM_STD_RATE_HZ, { 255, 4,  5, 255}}, \
    { (&htim8), PWM_STANDARD, PWM_STD_RATE_HZ, { 6,  7, 255, 255}}  \
  };
  init_status = pwm_.init(
    8, // Number of Channels
    3, // Numberf of timer blocks
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

  HAL_NVIC_EnableIRQ(BMI088_INT4_GYRO_EXTI_IRQn);  // EXTI4_IRQn Gyro DRDY Feedback
  HAL_NVIC_EnableIRQ(BMI088_INT1_ACCEL_EXTI_IRQn); // EXTI1_IRQn ACCEL DRDY
  HAL_NVIC_EnableIRQ(GPS_PPS_EXTI_IRQn);           // EXTI15_10_IRQn GPS PPD

  __HAL_UART_ENABLE_IT(gps_.huart(), UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(rc_.huart(), UART_IT_IDLE);

  telem_.rxStart(); // Also enables its interrupts.

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // High Rate Timer initialization

  misc_printf("\n\nPolling Timer Initialization\n");

  init_status = InitPollTimer(POLL_HTIM, POLL_HTIM_INSTANCE, POLL_TIM_CHANNEL);
  misc_exit_status(init_status);

  RED_LO;
  GRN_LO;
  BLU_LO;

#if SANDBOX
  misc_printf("\n\nStarting Sandbox\n");
  sandbox();
#else
  misc_printf("\n\nStarting Rosflight\n");
  verbose = false;
#endif
// clang-format on
}

