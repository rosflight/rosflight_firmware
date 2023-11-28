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

#include "stm32h7xx_hal.h"

#include <CubeMX.h>


#define EPOCH_HZ 	(400)
#define EPOCH_US 	(1000000/EPOCH_HZ)

#define FIFO_MIN_BUFFERS 2 // Defaults to double buffered with really slow data.


#define I2C_DMA_MAX_BUFFER_SIZE 64
#define SPI_DMA_MAX_BUFFER_SIZE 64

#define HTIM_LOW 							(&htim5)		// least significant word of 64-bit us timer
#define HTIM_LOW_INSTANCE			(TIM5)
#define HTIM_HIGH 						(&htim8)		// most  significant word of 64-bit us timer
#define HTIM_HIGH_INSTANCE		(TIM8)

#define POLL_HTIM 						(&htim7)	 	// High rate periodic interrupt timer (PITR)
#define POLL_TIM_CHANNEL 			TIM_CHANNEL_1
#define POLL_HTIM_INSTANCE		(TIM7)
#define POLLING_PERIOD_US     (100) // 100us, 10kHz
#define POLLING_FREQ_HZ       (1000000/POLLING_PERIOD_US) // 10000 Hz

// Pwm's
#define	PWM_CHANNELS					(10)			// Number of PWM output channels on the board
#define SERVO_PWM_CLK_DIV  		(200-1)   // 200MHz/200 = 1Hz or 1us per count
#define SERVO_PWM_PERIOD   		(20000)   // For 1000000Hz/50Hz = 20000
#define SERVO_PWM_CENTER    	(1500)   // 1us * 1500 = 1500 us
#define PWM_HTIM_0						(&htim1)
#define PWM_HTIM_1						(&htim1)
#define PWM_HTIM_2						(&htim1)
#define PWM_HTIM_3						(&htim1)
#define PWM_HTIM_4						(&htim4)
#define PWM_HTIM_5						(&htim4)
#define PWM_HTIM_6						(&htim4)
#define PWM_HTIM_7						(&htim4)
#define PWM_HTIM_8						(&htim3)
#define PWM_HTIM_9						(&htim3)
#define PWM_CHAN_0						(TIM_CHANNEL_1)
#define PWM_CHAN_1						(TIM_CHANNEL_2)
#define PWM_CHAN_2						(TIM_CHANNEL_3)
#define PWM_CHAN_3						(TIM_CHANNEL_4)
#define PWM_CHAN_4						(TIM_CHANNEL_3)
#define PWM_CHAN_5						(TIM_CHANNEL_2)
#define PWM_CHAN_6						(TIM_CHANNEL_1)
#define PWM_CHAN_7						(TIM_CHANNEL_4)
#define PWM_CHAN_8						(TIM_CHANNEL_1)
#define PWM_CHAN_9						(TIM_CHANNEL_2)
#define	PWM_MIN								(1000)
#define PWM_CENTER						(1500)
#define PWM_MAX								(2000)

// Imu1 is BMI088 on spi1
#define	BMI088_HZ 							(400) // 400, 1000, 2000 are the only options
#define BMI088_RANGE_A					(3) // 0,1,2,3 --> 3,6,12,24g
#define BMI088_RANGE_G					(2) // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
#define BMI088_FIFO_BUFFERS 		(FIFO_MIN_BUFFERS + BMI088_HZ/EPOCH_HZ)
#define IMU1_DRDY_PORT					(IMU1_ACCEL_DRDY_GPIO_Port)
#define IMU1_DRDY_PIN						(IMU1_ACCEL_DRDY_Pin)
//#define IMU1_DRDY_PORT_A				(IMU1_ACCEL_DRDY_GPIO_Port)
//#define IMU1_DRDY_PIN_A					(IMU1_ACCEL_DRDY_Pin)
//#define IMU1_DRDY_PORT_G				(IMU1_GYRO_DRDY_GPIO_Port)
//#define IMU1_DRDY_PIN_G					(IMU1_GYRO_DRDY_Pin)
#define IMU1_HZ 								(BMI088_HZ)
#define IMU1_SPI 								(&hspi1)
#define IMU1_CS_PORT_A					(IMU1_SPI1_CS_ACCEL_GPIO_Port)
#define IMU1_CS_PIN_A						(IMU1_SPI1_CS_ACCEL_Pin)
#define IMU1_CS_PORT_G					(IMU1_SPI1_CS_GYRO_GPIO_Port)
#define IMU1_CS_PIN_G						(IMU1_SPI1_CS_GYRO_Pin)
#define IMU1_RANGE_A						(BMI088_RANGE_A)
#define IMU1_RANGE_G						(BMI088_RANGE_G)

// Imu0 is ADIS on spi4
#define	ADIS165XX_HZ 						(400)
#define ADIS165XX_FIFO_BUFFERS 	(FIFO_MIN_BUFFERS + ADIS165XX_HZ/EPOCH_HZ)

#define IMU0_DRDY_PORT					(IMU0_DRDY_GPIO_Port)
#define IMU0_DRDY_PIN						(IMU0_DRDY_Pin)
#define IMU0_HZ 								(ADIS165XX_HZ)
#define IMU0_SPI 								(&hspi4)
#define IMU0_HTIM 							(&htim12) 			// ADIS 16500 ExtClk
#define IMU0_TIM_CHANNEL 				(TIM_CHANNEL_1) // ADIS 16500 ExtClk
#define IMU0_TIM_INSTANCE				(TIM12)					// ADIS 16500 ExtClk
#define IMU0_TIM_PERIOD_US			(500)   // 500 us, 2kHz
#define IMU0_CS_PORT						(IMU0_SPI4_CS_GPIO_Port)
#define IMU0_CS_PIN							(IMU0_SPI4_CS_Pin)
#define IMU0_RESET_PORT  				(IMU0_RST_GPIO_Port)
#define IMU0_RESET_PIN					(IMU0_RST_Pin)

// Pitot is DLHR on i2c1
#define DLHRL20G_HZ							(100)
#define DLHRL20G_FIFO_BUFFERS 	(FIFO_MIN_BUFFERS + DLHRL20G_HZ/EPOCH_HZ)
#define PITOT_DRDY_PORT					(PITOT_DRDY_GPIO_Port)
#define PITOT_DRDY_PIN					(PITOT_DRDY_Pin)

#define PITOT_HZ								(DLHRL20G_HZ)
#define PITOT_I2C 							(&hi2c1)
#define PITOT_I2C_ADDRESS 			(0x29)

// Baro is DPS310 on spi3
#define	DPS310_HZ 						(100)	// real value is lower
#define DPS310_FIFO_BUFFERS 	(FIFO_MIN_BUFFERS + DPS310_HZ/EPOCH_HZ)
#define BARO_DRDY_PORT				(BARO_DRDY_GPIO_Port)
#define BARO_DRDY_PIN					(BARO_DRDY_Pin)
#define BARO_CS_PORT					(BARO_CS_GPIO_Port)
#define BARO_CS_PIN						(BARO_CS_Pin)
#define BARO_HZ 							(DPS310_HZ)
#define BARO_SPI 							(&hspi3)

// Mag is IIS2MDC on spi2
#define	IIS2MDC_HZ 						(100)	// 10, 20, 50, 100 are the only options
#define IIS2MDC_FIFO_BUFFERS 	(FIFO_MIN_BUFFERS + IIS2MDC_HZ/EPOCH_HZ)
#define MAG_DRDY_PORT					(MAG_DRDY_GPIO_Port)
#define MAG_DRDY_PIN					(MAG_DRDY_Pin)
#define MAG_CS_PORT						(MAG_CS_GPIO_Port)
#define MAG_CS_PIN						(MAG_CS_Pin)
#define MAG_HZ 								(IIS2MDC_HZ)
#define MAG_SPI 							(&hspi2)

// SBus is on UART3
#define SBUS_HZ								(112) // 1000/9ms = 111.1Hz, 112 is rounds up
#define SBUS_FIFO_BUFFERS 		(FIFO_MIN_BUFFERS + SBUS_HZ/EPOCH_HZ)
#define SBUS_BAUD							(100000)
//
#define RC_HZ									(SBUS_HZ)
#define RC_UART								(&huart3)
#define RC_UART_INSTANCE			(USART3)
#define RC_UART_DMA						(&hdma_usart3_rx)
#define RC_BAUD								(SBUS_BAUD)

// uBlox on UART1
#define UBX_HZ								(10)
#define UBX_NUM								(3) // number of different types of packets
#define UBX_FIFO_BUFFERS 			(UBX_NUM*(FIFO_MIN_BUFFERS + UBX_HZ/EPOCH_HZ))
#define	UBX_BAUD							(115200)
//
#define GPS_HZ								(UBX_HZ)
#define GPS_PPS_PORT					(GPS_1PPS_GPIO_Port)
#define GPS_PPS_PIN						(GPS_1PPS_Pin)
#define GPS_UART							(&huart1)
#define GPS_UART_INSTANCE			(USART1)
#define GPS_UART_DMA					(&hdma_usart1_rx)
#define	GPS_BAUD							(115200)

// Telemetry UART & VCP
// Serial
#define SERIAL_HZ										(IMU0_HZ)
#define SERIAL_QOS_FIFOS 						(3)
#define SERIAL_TX_FIFO_BUFFERS			(PACKET_FIFO_MAX_BUFFERS)
#define SERIAL_RX_FIFO_BUFFER_BYTES (4096)
// Telem (USART2)
#define TELEM_HZ										(SERIAL_HZ)
#define	TELEM_BAUD									(921600)
#define TELEM_UART									(&huart2)
#define TELEM_UART_INSTANCE					(USART2)
#define TELEM_UART_DMA							(0) //(&hdma_usart2_rx)
// VCP
#define VCP_HZ											(SERIAL_HZ)

// SDMMC
#define SD_HSD 								(&hsd1)
#define SD_HSD_INSTANCE 			(SDMMC1)

// Onboard ADC's
#define ADC_HZ								(10) // Maximum is 500 Hz.
#define ADC_FIFO_BUFFERS			(FIFO_MIN_BUFFERS)

#define ADC_ADC_EXTERNAL  		(&hadc1)
#define ADC_ADC_INSTANCE_EXTERNAL			(ADC1)
#define ADC_CHANNELS_EXT			(6)
#define ADC_BATTERY_VOLTS			(5)
#define ADC_BATTERY_CURR			(4)
#define ADC_STM_5V0						(3)
#define ADC_STM_12V						(2)
#define ADC_SERVO_VOLTS				(1)
#define ADC_JETSON_3V3				(0)

#define ADC_ADC_INTERNAL			(&hadc3)
#define ADC_ADC_INSTANCE_INTERNAL	(ADC3)
#define ADC_CHANNELS_INT			(3)
#define ADC_STM_TEMPERATURE 	(0)
#define ADC_STM_VBAT 					(1)
#define ADC_STM_VREFINT 			(2)

// Red LED PE7
#define RED_HI  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7, GPIO_PIN_SET)
#define RED_LO  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7, GPIO_PIN_RESET)
#define RED_TOG HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7)

// Green LED PE15
#define GRN_HI  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_SET)
#define GRN_LO  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_RESET)
#define GRN_TOG HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_15)

// Blue LED PE8
#define BLU_HI  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8, GPIO_PIN_SET)
#define BLU_LO  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8, GPIO_PIN_RESET)
#define BLU_TOG HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8)


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern CRC_HandleTypeDef hcrc;

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

extern RNG_HandleTypeDef hrng;

extern RTC_HandleTypeDef hrtc;

extern SD_HandleTypeDef hsd1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#if 1 // probes
	// CN11 34 TP5
	#define PB0_HI  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_SET)
	#define PB0_LO  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_RESET)
	#define PB0_TOG HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0)

	// CN12 19 TP6
	#define PC7_HI  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET)
	#define PC7_LO  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_RESET)
	#define PC7_TOG HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7)

	// CN11 47 (Jetson DRDY)
	#define PE5_HI  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_SET)
	#define PE5_LO  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_RESET)
	#define PE5_TOG HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5)

	// Real Board - PH1 J105 pin 18 RST
	#define PH1_HI  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_1, GPIO_PIN_SET)
	#define PH1_LO  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_1, GPIO_PIN_RESET)
	#define PH1_TOG HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_1)

	// Real Board - PE3 J105 pin 19 CS
	#define PE3_HI  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET)
	#define PE3_LO  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET)
	#define PE3_TOG HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3)

	// Real Board - PB1 J105 pin 23/25 Sync Bus (CAN XCVR)
	#define PB1_HI  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET)
	#define PB1_LO  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_RESET)
	#define PB1_TOG HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1)

	// PB15
	#define PB15_HI  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET)
	#define PB15_LO  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET)
	#define PB15_TOG HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15)

#else
	// CN11 34 TP5
	#define PB0_HI
	#define PB0_LO
	#define PB0_TOG

	// CN12 19 TP6
	#define PC7_HI
	#define PC7_LO
	#define PC7_TOG

	// CN11 47 (Jetson DRDY)
	#define PE5_HI
	#define PE5_LO
	#define PE5_TOG _5)

	// Real Board - PH1 J105 pin 18 RST
	#define PH1_HI
	#define PH1_LO
	#define PH1_TOG

	// Real Board - PE3 J105 pin 19 CS
	#define PE3_HI
	#define PE3_LO
	#define PE3_TOG

	// Real Board - PB1 J105 pin 23/25 Sync Bus (CAN XCVR)
	#define PB1_HI
	#define PB1_LO
	#define PB1_TOG

	// PB15
	#define PB15_HI
	#define PB15_LO
	#define PB15_TOG
#endif

#define DRIVER_OK						 		(0x00000000)
#define DRIVER_ID_MISMATCH 			(0x00000002)
#define DRIVER_SELF_DIAG_ERROR 	(0x00000004)
#define DRIVER_HAL_ERROR 				(0x00000008)
#define DRIVER_HAL_ERROR2 			(0x00000010)
#define DRIVER_FIFO_INIT_ERROR 	(0x00000020)
#define UBX_ACK									(0x00000040)
#define UBX_NAK									(0x00000080)
#define UBX_ACKNAK_FAIL					(0x00000100)
#define UBX_SUCCESS							(0x00000200)
#define UBX_FAIL_BAUD_CHANGE		(0x00000400)

#endif /* BOARDCONFIG_H_ */
