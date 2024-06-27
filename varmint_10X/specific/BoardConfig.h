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

//#define SANDBOX // set this via CMAKE

#include "stm32h7xx_hal.h"

#include <main.h>

// UART used for printf's
#define MISC_HUART (&huart2)

/////////////////////////////////////////////////////////////////////////////////////////////
// MiddleWare name : AL94.I-CUBE-USBD-COMPOSITE.1.0.3
//
#define VCP_Transmit(buffer, length) CDC_Transmit(0, buffer, length)
#define _USBD_USE_HS false
extern PCD_HandleTypeDef hpcd_USB_OTG_FS; // USB FS (48 MB/s)

#define _USBD_USE_CDC_ACM true // /dev/ttyACM* device type
#define _USBD_CDC_ACM_COUNT 1
#define _USBD_USE_CDC_RNDIS false
#define _USBD_USE_CDC_ECM false
#define _USBD_USE_HID_MOUSE false
#define _USBD_USE_HID_KEYBOARD false
#define _USBD_USE_HID_CUSTOM false
#define _USBD_USE_UAC_MIC false
#define _USBD_USE_UAC_SPKR false
#define _USBD_USE_UVC false
#define _USBD_USE_MSC false
#define _USBD_USE_DFU false
#define _USBD_USE_PRNTR false
#define _STM32F1_DEVICE false // F1 devices are a special case
//
// End MiddleWare name : AL94.I-CUBE-USBD-COMPOSITE.1.0.3
/////////////////////////////////////////////////////////////////////////////////////////////

#define EPOCH_HZ (400)
#define EPOCH_US (1000000 / EPOCH_HZ)

#define FIFO_MIN_BUFFERS 2 // Defaults to double buffered with really slow data.

#define I2C_DMA_MAX_BUFFER_SIZE 64
#define SPI_DMA_MAX_BUFFER_SIZE 64

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

// Pwm's
#define PWM_MIN (1000)
#define PWM_CENTER (1500)
#define PWM_MAX (2000)

// TIMER, CHAN, PWM_STD|PWM_DSHOT, 50|300|600, ...
// for PWM_STD, Hz, for DSHOT, bps
// PWM_STD or PWM_DSHOT
#define PWM_DSHOT_PERIOD_COUNTS (667)		// 300000 bps 667 counts per bit at 200MHz clock
#define PWM_MKS_PERIOD_COUNTS   (3000)		// 333Hz at 1MHz clock
#define PWM_STD_PERIOD_COUNTS   (20000)   	// 50 Hz at 1MHz clock

#define PWM_BLOCKS 3
#define PWM_BLOCKS_DEFINE \
{ \
	{ TIM1, PWM_STD, PWM_STD_PERIOD_COUNTS}, \
	{ TIM4, PWM_STD, PWM_STD_PERIOD_COUNTS}, \
	{ TIM3, PWM_STD, PWM_STD_PERIOD_COUNTS} \
}

#define PWM_CHANNELS (10) // Number of PWM output channels on the board
#define PWM_CHANNELS_DEFINE \
{ \
	{ (&htim1), TIM_CHANNEL_1, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim1), TIM_CHANNEL_2, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim1), TIM_CHANNEL_3, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim1), TIM_CHANNEL_4, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim4), TIM_CHANNEL_3, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim4), TIM_CHANNEL_2, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim4), TIM_CHANNEL_1, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim4), TIM_CHANNEL_4, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim3), TIM_CHANNEL_1, PWM_MIN, PWM_CENTER, PWM_MAX}, \
	{ (&htim3), TIM_CHANNEL_2, PWM_MIN, PWM_CENTER, PWM_MAX}  \
}

// BMI088 IMU
#define BMI088_SPI (&hspi1)

#define BMI088_HZ (EPOCH_HZ) // 400, 1000, 2000 are the only options
#define BMI088_RANGE_A (3)   // 0,1,2,3 --> 3,6,12,24g for BMI088; 2 4 8 16g for BMI 085
#define BMI088_RANGE_G (2)   // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
#define BMI088_FIFO_BUFFERS (FIFO_MIN_BUFFERS + BMI088_HZ / EPOCH_HZ)

// ADIS IMU
#define ADIS165XX_HZ (EPOCH_HZ)
#define ADIS165XX_FIFO_BUFFERS (FIFO_MIN_BUFFERS + ADIS165XX_HZ / EPOCH_HZ)
#define ADIS165XX_SPI (&hspi4)

#define ADIS165XX_HTIM (&htim12)              // ADIS 16500 ExtClk
#define ADIS165XX_TIM_CHANNEL (TIM_CHANNEL_1) // ADIS 16500 ExtClk
#define ADIS165XX_TIM_INSTANCE (TIM12)        // ADIS 16500 ExtClk
#define ADIS165XX_TIM_PERIOD_US (500)         // 500 us, 2kHz

// DLHR Pitot is on i2c1
#define DLHRL20G_HZ (100)
#define DLHRL20G_FIFO_BUFFERS (FIFO_MIN_BUFFERS + DLHRL20G_HZ / EPOCH_HZ)

#define PITOT_DRDY_PORT (PITOT_DRDY_GPIO_Port)
#define PITOT_DRDY_PIN (PITOT_DRDY_Pin)

#define PITOT_HZ (DLHRL20G_HZ)
#define PITOT_I2C (&hi2c1)
#define PITOT_I2C_ADDRESS (DLHRL20G_I2C_ADDRESS)

// MS4525D Pitot
#define MS4525_HZ (100)
#define MS4525_FIFO_BUFFERS (FIFO_MIN_BUFFERS + MS4525_HZ / EPOCH_HZ)

// AUAV is both baro (absolute) and differtial (Pitot)
#define AUAV_SPI (&hspi4)
#define AUAV_HZ (100)
// Absolute (Baro)
#define AUAV_BARO_HZ (AUAV_HZ) // real value is lower
#define AUAV_BARO_FIFO_BUFFERS (FIFO_MIN_BUFFERS + AUAV_BARO_HZ / EPOCH_HZ)

// Differential (Pitot)
#define AUAV_PITOT_HZ (AUAV_HZ) // real value is lower
#define AUAV_PITOT_FIFO_BUFFERS (FIFO_MIN_BUFFERS + AUAV_PITOT_HZ / EPOCH_HZ)

// Digital Potentiometer used in later versions
//	#define MCP4017_I2C_ADDRESS 		(0x2F)
#define MCP4017_I2C (&hi2c1)
#define SERVO_VOLTAGE (4.8) // Volts

// I2C EEPROM in 11X
#define EEPROM_I2C (&hi2c1)
#define EEPROM_I2C_ADDRESS (0x50)

#define DPS310_SPI (&hspi3)
#define DPS310_3_WIRE (true)
#define IIS2MDC_SPI (&hspi2)

// Baro is DPS310
#define DPS310_HZ (50) // up to 50 Hz.
#define DPS310_FIFO_BUFFERS (FIFO_MIN_BUFFERS + DPS310_HZ / EPOCH_HZ)

// Mag is IIS2MDC
// HZ no faster than 100Hz. 10, 20, 50, 100 are the only options for continuous mode
#define IIS2MDC_HZ (100)
#define IIS2MDC_FIFO_BUFFERS (FIFO_MIN_BUFFERS + IIS2MDC_HZ / EPOCH_HZ)

// Mag IST8308 (pixracer Pro)
// HZ no faster than 100Hz
#define IST3808_HZ (100)
#define IST8308_FIFO_BUFFERS (FIFO_MIN_BUFFERS + IST3808_HZ / EPOCH_HZ)
#define IST3808_I2C (&hi2c1)
#define IST3808_I2C_ADDRESS (0X0C)

// SBus is on UART3 for Varmints, UART6 for PixRacer Pro
#define SBUS_HZ (112) // 1000/9ms = 111.1Hz, 112 is rounds up
#define SBUS_FIFO_BUFFERS (FIFO_MIN_BUFFERS + SBUS_HZ / EPOCH_HZ)
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
#define UBX_FIFO_BUFFERS (UBX_NUM * (FIFO_MIN_BUFFERS + UBX_HZ / EPOCH_HZ))
//#define	UBX_BAUD					(57600)

//
#define GPS_HZ (UBX_HZ)
#define GPS_BAUD (57600)

#define UBX_PROTOCOL (UBX_M8)
#define GPS_HAS_PPS (true)
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

// SDMMC
#define SD_HSD (&hsd1)
#define SD_HSD_INSTANCE (SDMMC1)

// Onboard ADC's
#define ADC_HZ (10) // Maximum is 500 Hz.
#define ADC_FIFO_BUFFERS (FIFO_MIN_BUFFERS)

#define ADC_ADC_EXTERNAL (&hadc1)
#define ADC_ADC_INSTANCE_EXTERNAL (ADC1)
#define ADC_EXT_DMA_RAM "my_dma_buffers"
#define ADC_CHANNELS_EXT (6)

#define ADC_CC_3V3 (0)          // INP 4
#define ADC_SERVO_VOLTS (1)     // INP 7
#define ADC_12V (2)             // INP 8
#define ADC_5V0 (3)             // INP 10
#define ADC_BATTERY_CURRENT (4) // INP 11
#define ADC_BATTERY_VOLTS (5)   // INP 16

#define ADC_ADC_INTERNAL (&hadc3)
#define ADC_ADC_INSTANCE_INTERNAL (ADC3)

#define ADC_INT_DMA_RAM "my_bdma_buffers" // NOTE! ADC3 using BDMA so this needs to be in SRAM4
#define ADC_CHANNELS_INT (3)

#define ADC_STM_TEMPERATURE (0 + ADC_CHANNELS_EXT) // INP 18 (Internal)
#define ADC_STM_VBAT (1 + ADC_CHANNELS_EXT)        // INP 17 (Internal)
#define ADC_STM_VREFINT (2 + ADC_CHANNELS_EXT)     // INP 19 (Internal)

// NOTE! This lets us put all the config in one file
#define ADC_CFG_CHANS_DEFINE                                                                                           \
    {                                                                                                                  \
        {ADC_REGULAR_RANK_1, ADC_CHANNEL_4, 1.100, 0.0},              /* ADC_CC_3V3 */                                 \
            {ADC_REGULAR_RANK_2, ADC_CHANNEL_7, 2.690, 0.0},          /* ADC_SERVO_VOLTS */                            \
            {ADC_REGULAR_RANK_3, ADC_CHANNEL_8, 4.010, 0.0},          /* ADC_12V */                                    \
            {ADC_REGULAR_RANK_4, ADC_CHANNEL_10, 1.680, 0.0},         /* ADC_5V0 */                                    \
            {ADC_REGULAR_RANK_5, ADC_CHANNEL_11, 10.000, 0.0},        /* ADC_BATTERY_CURRENT */                        \
            {ADC_REGULAR_RANK_6, ADC_CHANNEL_16, 11.215, 0.0},        /* ADC_BATTERY_VOLTS */                          \
            {ADC_REGULAR_RANK_1, ADC_CHANNEL_TEMPSENSOR, 1.000, 0.0}, /* ADC_STM_TEMPERATURE */                        \
            {ADC_REGULAR_RANK_2, ADC_CHANNEL_VBAT, 4.000, 0.0},       /* ADC_STM_VBAT */                               \
        {                                                                                                              \
            ADC_REGULAR_RANK_3, ADC_CHANNEL_VREFINT, 1.000, 0.0                                                        \
        } /* ADC_STM_VREFINT */                                                                                        \
    }

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

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern CRC_HandleTypeDef hcrc;

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

// extern RNG_HandleTypeDef hrng;

extern RTC_HandleTypeDef hrtc;

extern SD_HandleTypeDef hsd1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi6;

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi5_tx;
extern DMA_HandleTypeDef hdma_spi6_rx;
extern DMA_HandleTypeDef hdma_spi6_tx;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;

// Probes
#if 1
// CN11 34 TP5
#define PB0_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define PB0_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define PB0_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0)

// CN12 19 TP6
#define PC7_HI HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define PC7_LO HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define PC7_TOG HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7)

// CN11 47 (Jetson DRDY)
#define PE5_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET)
#define PE5_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET)
#define PE5_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5)

// Real Board - PH1 J105 pin 18 RST
#define PH1_HI HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET)
#define PH1_LO HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET)
#define PH1_TOG HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1)

// Real Board - PE3 J105 pin 19 CS
#define PE3_HI HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define PE3_LO HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define PE3_TOG HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3)

// Real Board - PB1 J105 pin 23/25 Sync Bus (CAN XCVR)
#define PB1_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define PB1_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define PB1_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)

// PB15
#define PB15_HI HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define PB15_LO HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define PB15_TOG HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15)

#define PROBE1_HI PE3_HI
#define PROBE1_LO PE3_LO
#define PROBE1_TOG PE3_TOG

#define PROBE2_HI PH1_HI
#define PROBE2_LO PH1_LO
#define PROBE2_TOG PH1_TOG

#define PROBE3_HI PB15_HI
#define PROBE3_LO PB15_LO
#define PROBE3_TOG PB15_TOG

#define PROBE4_HI PB1_HI
#define PROBE4_LO PB1_LO
#define PROBE4_TOG PB1_TOG

#else

#define PROBE1_HI                                                                                                      \
    {}
#define PROBE1_LO                                                                                                      \
    {}
#define PROBE1_TOG                                                                                                     \
    {}

#define PROBE2_HI                                                                                                      \
    {}
#define PROBE2_LO                                                                                                      \
    {}
#define PROBE2_TOG                                                                                                     \
    {}

#define PROBE3_HI                                                                                                      \
    {}
#define PROBE3_LO                                                                                                      \
    {}
#define PROBE3_TOG                                                                                                     \
    {}

#define PROBE4_HI                                                                                                      \
    {}
#define PROBE4_LO                                                                                                      \
    {}
#define PROBE4_TOG                                                                                                     \
    {}

#endif

#define DRIVER_OK (0x00000000)
#define DRIVER_ID_MISMATCH (0x00000002)
#define DRIVER_SELF_DIAG_ERROR (0x00000004)
#define DRIVER_HAL_ERROR (0x00000008)
#define DRIVER_HAL_ERROR2 (0x00000010)
#define DRIVER_FIFO_INIT_ERROR (0x00000020)
#define UBX_ACK (0x00000040)
#define UBX_NAK (0x00000080)
#define UBX_ACKNAK_FAIL (0x00000100)
#define UBX_SUCCESS (0x00000200)
#define UBX_FAIL_BAUD_CHANGE (0x00000400)
#define VOLTAGE_SET_FAIL (0x00000800)
#define TIMERS_INVALID (0x00001000)

#endif /* BOARDCONFIG_H_ */
