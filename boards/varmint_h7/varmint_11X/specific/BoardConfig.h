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

#include <CommonConfig.h>

#define SANDBOX false
#define BOARD_STATUS_PRINT false
#define USE_TELEM 0 // 1 = use UART, 0 = use VCP for link to companion computer.

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
  Sbus rc_;                 /* All */ \
  Ubx gps_;                 /* All */ \
  Adc adc_;                 /* All */ \
  Telem telem_;             /* All */ \
  Vcp vcp_;                 /* All */ \
  Pwm pwm_;                 /* All */ \
  Sd sd_;                   /* All */ \
  Dps310 baro_;             /* All */ \
  Adis165xx imu0_;          /* Varmint 10-12X */ \
  Bmi088 imu1_;             /* Varmint 10-12X */ \
  Iis2mdc mag_;             /* Varmint 10-12X */ \
  DlhrL20G pitot_;          /* Varmint 10-11X */ \
  Mcp4017 servoV_;          /* Varmint 11-12X */ \
  Lidarlitev3hp range_;    /* External I2C */ \
  /*		Auav pitot_; */     /* Varmint 12X */ \
  /*		Auav baro2_; */     /* Varmint 12X */ \
  /*		Bmi088 imu0_;    */ /* PixRacer Pro */ \
  /*		Ms4525 pitot_; */   /* PixRacer Pro */ \
  /*		Ist8308 mag_;  */   /* PixRacer Pro */ \
  /**/
// clang-format on


// 48-bit us counter.
// Prefer to have the 32-bit counter on the low order bytes:
#define HTIM_LOW (&htim5) // 32-bit counter
#define HTIM_LOW_INSTANCE (TIM5)
#define HTIM_HIGH (&htim8) // 16-bit overflow counter
#define HTIM_HIGH_INSTANCE (TIM8)

#define POLL_HTIM (&htim7) // High rate periodic interrupt timer (PITR)
#define POLL_TIM_CHANNEL TIM_CHANNEL_1
#define POLL_HTIM_INSTANCE (TIM7)
#define POLLING_PERIOD_US (100)                       // 100us, 10kHz
#define POLLING_FREQ_HZ (1000000 / POLLING_PERIOD_US) // 10000 Hz

///////////////////////////////////////////////////////////////////////////////////////////////////
// Pwm's
//#define PWM_SERVO_MIN 	  	(1000)
//#define PWM_SERVO_MAX 	  	(2000)
//
//#define DSHOT_ESC_MIN		(48)
//#define DSHOT_ESC_MAX		(2047)
//
//#define PWM_DSHOT_RATE_HZ 	(300000.0) // baud rate
//#define PWM_MKS_RATE_HZ 	(333.0)
//#define PWM_STD_RATE_HZ 	(50.0)

#define PWM_CHANNELS (10) // Number of PWM output channels on the board
#define PWM_TIMER_BLOCKS (3)

//typedef enum : uint8_t
//{
//	PWM_STANDARD,
//	PWM_DSHOT
//} pwm_type;

// Arrays are the mapping of CH1-4 to the 10 PWM Channels Index
// clang-format off
#define PWM_INIT_DEFINE \
{ \
  {(&htim1), PWM_STANDARD, PWM_STD_RATE_HZ, {0, 1, 2, 3}}, \
  {(&htim4), PWM_STANDARD, PWM_STD_RATE_HZ, {6, 5, 4, 7}}, \
  {(&htim3), PWM_STANDARD, PWM_STD_RATE_HZ, { 8, 9, 255, 255 }} \
}
// clang-format on

// Channel order based on hardware pinout naming
//	TIMER 1 TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
//	TIMER 4 TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_4,
//	TIMER 3 TIM_CHANNEL_1, TIM_CHANNEL_2

// BMI088 IMU
#define BMI088_SPI (&hspi1)

#define BMI088_HZ (EPOCH_HZ) // 400, 1000, 2000 are the only options
#define BMI088_RANGE_A (3)   // 0,1,2,3 --> 3,6,12,24g for BMI088; 2 4 8 16g for BMI 085
#define BMI088_RANGE_G (2)   // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
#define BMI088_ROTATION (const double[]){-1.0, 0.0, 0.0,   0.0, -1.0, 0.0,    0.0, 0.0, 1.0}

// ADIS IMU
#define ADIS165XX_HZ (EPOCH_HZ)
#define ADIS165XX_SPI (&hspi4)

#define ADIS165XX_HTIM (&htim12)              // ADIS 16500 ExtClk
#define ADIS165XX_TIM_CHANNEL (TIM_CHANNEL_1) // ADIS 16500 ExtClk
#define ADIS165XX_TIM_INSTANCE (TIM12)        // ADIS 16500 ExtClk
#define ADIS165XX_TIM_PERIOD_US (500)         // 500 us, 2kHz
#define ADIS165XX_ROTATION (const double[]){-1.0, 0.0, 0.0,   0.0, -1.0, 0.0,    0.0, 0.0, 1.0}

// Range Lidar Sensor on i2c2
#define LIDAR_HZ (100)
#define LIDAR_I2C (&hi2c2)
#define LIDAR_I2C_ADDRESS (LIDARLITEV3HP_ADDRESS)

// DLHR Pitot is on i2c1
#define DLHRL20G_HZ (100)

#define PITOT_DRDY_PORT (PITOT_DRDY_GPIO_Port)
#define PITOT_DRDY_PIN (PITOT_DRDY_Pin)

#define PITOT_HZ (DLHRL20G_HZ)
#define PITOT_I2C (&hi2c1)
#define PITOT_I2C_ADDRESS (DLHRL20G_I2C_ADDRESS)

// MS4525D Pitot
#define MS4525_HZ (100)

// AUAV is both baro (absolute) and differtial (Pitot)
#define AUAV_SPI (&hspi4)
#define AUAV_HZ (100)
// Absolute (Baro)
#define AUAV_BARO_HZ (AUAV_HZ) // real value is lower

// Differential (Pitot)
#define AUAV_PITOT_HZ (AUAV_HZ) // real value is lower

// Digital Potentiometer used in later versions
//	#define MCP4017_I2C_ADDRESS 		(0x2F)
#define MCP4017_I2C (&hi2c1)
#define SERVO_VOLTAGE (4.8) // Volts

// I2C EEPROM in 11X
#define EEPROM_I2C (&hi2c1)
#define EEPROM_I2C_ADDRESS (0x50)

#define DPS310_SPI (&hspi2)
#define DPS310_3_WIRE (true)
#define IIS2MDC_SPI (&hspi2)

// Baro is DPS310
#define DPS310_HZ (50) // up to 50 Hz.

// Mag is IIS2MDC
// HZ no faster than 100Hz. 10, 20, 50, 100 are the only options for continuous mode
#define IIS2MDC_HZ (100)
#define IIS2MDC_ROTATION (const double[]){-1.0, 0.0, 0.0,   0.0, 1.0, 0.0,    0.0, 0.0, -1.0}

// Mag IST8308 (pixracer Pro)
// HZ no faster than 100Hz
#define IST3808_HZ (100)
#define IST3808_I2C (&hi2c1)
#define IST3808_I2C_ADDRESS (0X0C)
#define IST3808_ROTATION (const double[]){1.0, 0.0, 0.0,   0.0, 1.0, 0.0,    0.0, 0.0, 1.0}

// SBus is on UART3 for Varmints, UART6 for PixRacer Pro
#define SBUS_HZ (112) // 1000/9ms = 111.1Hz, 112 is rounds up
#define SBUS_BAUD (100000)
//
#define RC_HZ (SBUS_HZ)
#define RC_BAUD (SBUS_BAUD)
#define RC_UART (&huart3)
#define RC_UART_INSTANCE (USART3)
#define RC_UART_DMA (&hdma_usart3_rx)

// uBlox
#define UBX_HZ (10)
#define UBX_NUM (3) // number of different types of packets
//#define	UBX_BAUD					(57600)

#define GPS_HZ (UBX_HZ)
#define GPS_BAUD (57600)

#define GPS_PPS_PORT (GPS_1PPS_GPIO_Port)
#define GPS_PPS_PIN (GPS_1PPS_Pin)
#define GPS_UART (&huart1)
#define GPS_UART_INSTANCE (USART1)
#define GPS_UART_DMA (&hdma_usart1_rx)

// Telemetry UART2 & VCP
// Serial
#define SERIAL_HZ (EPOCH_HZ) // Loop time is driven by IMU period.
#define SERIAL_QOS_FIFOS (3)
#define SERIAL_TX_FIFO_BUFFERS (PACKET_FIFO_MAX_BUFFERS)
#define SERIAL_RX_FIFO_BUFFER_BYTES (4096)
// Telem (USART2)
#define TELEM_HZ (SERIAL_HZ)
#define TELEM_BAUD (921600) //(57600)
#define TELEM_UART (&huart2)
#define TELEM_UART_INSTANCE (USART2)
#define TELEM_UART_DMA (0) //(&hdma_usart2_rx)
// VCP
#define VCP_HZ (SERIAL_HZ)

// Onboard ADC's
#define ADC_HZ (10) // Maximum is 500 Hz.

#define ADC_ADC_EXTERNAL (&hadc1)
#define ADC_ADC_INSTANCE_EXTERNAL (ADC1)
#define ADC_EXT_DMA_RAM DMA_RAM
#define ADC_CHANNELS_EXT (6)

#define ADC_CC_3V3 (0)          // INP 4
#define ADC_SERVO_VOLTS (1)     // INP 7
#define ADC_12V (2)             // INP 8
#define ADC_5V0 (3)             // INP 10
#define ADC_BATTERY_CURRENT (4) // INP 11
#define ADC_BATTERY_VOLTS (5)   // INP 16

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
  {ADC_REGULAR_RANK_1, ADC_CHANNEL_4, 1.100, 0.0},            /* ADC_CC_3V3 */ \
  {ADC_REGULAR_RANK_2, ADC_CHANNEL_7, 2.690, 0.0},          /* ADC_SERVO_VOLTS */ \
  {ADC_REGULAR_RANK_3, ADC_CHANNEL_8, 4.010, 0.0},          /* ADC_12V */ \
  {ADC_REGULAR_RANK_4, ADC_CHANNEL_10, 1.680, 0.0},         /* ADC_5V0 */ \
  {ADC_REGULAR_RANK_5, ADC_CHANNEL_11, 10.000, 0.0},        /* ADC_BATTERY_CURRENT */ \
  {ADC_REGULAR_RANK_6, ADC_CHANNEL_16, 11.215, 0.0},        /* ADC_BATTERY_VOLTS */ \
  {ADC_REGULAR_RANK_1, ADC_CHANNEL_TEMPSENSOR, 1.000, 0.0}, /* ADC_STM_TEMPERATURE */ \
  {ADC_REGULAR_RANK_2, ADC_CHANNEL_VBAT, 4.000, 0.0},       /* ADC_STM_VBAT */ \
  {ADC_REGULAR_RANK_3, ADC_CHANNEL_VREFINT, 1.000, 0.0} /* ADC_STM_VREFINT */ \
}
// clang-format on

#define ADC_CHANNELS (ADC_CHANNELS_EXT + ADC_CHANNELS_INT)

// Red LED
// PE7
#define RED_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET)
#define RED_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET)
#define RED_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7)

// Green LED
// PE15
#define GRN_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)
#define GRN_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)
#define GRN_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15)

// Blue LED
// PE8
#define BLU_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET)
#define BLU_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET)
#define BLU_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8)

// Probes
#if 1

// Real Board - PE3 J105 pin 19 CS
#define PROBE1_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define PROBE1_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define PROBE1_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3)

// Real Board - PH1 J105 pin 18 RST
#define PROBE2_HI HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET)
#define PROBE2_LO HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET)
#define PROBE2_TOG HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1)

// PB15
#define PROBE3_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define PROBE3_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define PROBE3_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15)

// Real Board - PB1 J105 pin 23/25 Sync Bus (CAN XCVR)
#define PROBE4_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define PROBE4_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define PROBE4_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)

// CN11 34 TP5 PB0
#define PB0_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define PB0_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define PB0_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0)

// CN12 19 TP6 PC7
#define PC7_HI HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define PC7_LO HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define PC7_TOG HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7)

// CN11 47 (Jetson DRDY) PE5
#define PE5_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET)
#define PE5_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET)
#define PE5_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5)

#else

#define PROBE1_HI
#define PROBE1_LO
#define PROBE1_TOG

#define PROBE2_HI
#define PROBE2_LO
#define PROBE2_TOG

#define PROBE3_HI
#define PROBE3_LO
#define PROBE3_TOG

#define PROBE4_HI
#define PROBE4_LO
#define PROBE4_TOG
#endif

#endif /* BOARDCONFIG_H_ */
