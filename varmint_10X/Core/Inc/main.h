/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	void SystemClock_Config(void);
	void PeriphCommonClock_Config(void);
	void MPU_Initialize(void);
	void MPU_Config(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_BDMA_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_SPI3_Init(void);
void MX_SPI4_Init(void);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_TIM7_Init(void);
void MX_TIM12_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_ADC1_Init(void);
void MX_ADC3_Init(void);
void MX_FDCAN1_Init(void);
void MX_SDMMC1_SD_Init(void);
void MX_RTC_Init(void);
void MX_CRC_Init(void);
void MX_RNG_Init(void);
void MX_I2C2_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define J000_JETSON_32KHZ_CLK_Pin GPIO_PIN_14
#define J000_JETSON_32KHZ_CLK_GPIO_Port GPIOC
#define J000_JETSON_SYNC_Pin GPIO_PIN_13
#define J000_JETSON_SYNC_GPIO_Port GPIOC
#define J000_JETSON_SYNC_EXTI_IRQn EXTI15_10_IRQn
#define ADIS165XX_SCK_Pin GPIO_PIN_2
#define ADIS165XX_SCK_GPIO_Port GPIOE
#define PITOT_I2C1_SDA_Pin GPIO_PIN_9
#define PITOT_I2C1_SDA_GPIO_Port GPIOB
#define GPS_UART1_RX_Pin GPIO_PIN_7
#define GPS_UART1_RX_GPIO_Port GPIOB
#define BARO_MISO_Pin GPIO_PIN_4
#define BARO_MISO_GPIO_Port GPIOB
#define BARO_SCK_Pin GPIO_PIN_3
#define BARO_SCK_GPIO_Port GPIOB
#define BMI088_ACCEL_DRDY_Pin GPIO_PIN_15
#define BMI088_ACCEL_DRDY_GPIO_Port GPIOA
#define BMI088_ACCEL_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define J101_DEBUG_SWCLK_Pin GPIO_PIN_14
#define J101_DEBUG_SWCLK_GPIO_Port GPIOA
#define J101_DEBUG_SWDIO_Pin GPIO_PIN_13
#define J101_DEBUG_SWDIO_GPIO_Port GPIOA
#define TP4_Pin GPIO_PIN_15
#define TP4_GPIO_Port GPIOC
#define J105_2_SPI4_EXT_CS_Pin GPIO_PIN_3
#define J105_2_SPI4_EXT_CS_GPIO_Port GPIOE
#define ADIS165XX_DRDY_Pin GPIO_PIN_8
#define ADIS165XX_DRDY_GPIO_Port GPIOB
#define ADIS165XX_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define PITOT_I2C1_SCL_Pin GPIO_PIN_6
#define PITOT_I2C1_SCL_GPIO_Port GPIOB
#define J103_1_TELEM_UART2_TX_Pin GPIO_PIN_5
#define J103_1_TELEM_UART2_TX_GPIO_Port GPIOD
#define J102_SDMMC1_CMD_Pin GPIO_PIN_2
#define J102_SDMMC1_CMD_GPIO_Port GPIOD
#define J102_SDMMC1_D3_Pin GPIO_PIN_11
#define J102_SDMMC1_D3_GPIO_Port GPIOC
#define J102_SDMMC1_D2_Pin GPIO_PIN_10
#define J102_SDMMC1_D2_GPIO_Port GPIOC
#define J000_JETSON_USB_P_Pin GPIO_PIN_12
#define J000_JETSON_USB_P_GPIO_Port GPIOA
#define EXT_OSC_Pin GPIO_PIN_0
#define EXT_OSC_GPIO_Port GPIOH
#define ADIS165XX_CSn_Pin GPIO_PIN_4
#define ADIS165XX_CSn_GPIO_Port GPIOE
#define DPS310_CSn_Pin GPIO_PIN_1
#define DPS310_CSn_GPIO_Port GPIOE
#define J106_PWM10_Pin GPIO_PIN_5
#define J106_PWM10_GPIO_Port GPIOB
#define J103_1_TELEM_UART2_RX_Pin GPIO_PIN_6
#define J103_1_TELEM_UART2_RX_GPIO_Port GPIOD
#define J103_1_TELEM_UART2_CTS_Pin GPIO_PIN_3
#define J103_1_TELEM_UART2_CTS_GPIO_Port GPIOD
#define J102_SDMMC1_CK_Pin GPIO_PIN_12
#define J102_SDMMC1_CK_GPIO_Port GPIOC
#define GPS_UART1_TX_Pin GPIO_PIN_9
#define GPS_UART1_TX_GPIO_Port GPIOA
#define J000_JETSON_USB_N_Pin GPIO_PIN_11
#define J000_JETSON_USB_N_GPIO_Port GPIOA
#define J105_2_SPI4_EXT_RST_Pin GPIO_PIN_1
#define J105_2_SPI4_EXT_RST_GPIO_Port GPIOH
#define J000_JETSON_DRDY_Pin GPIO_PIN_5
#define J000_JETSON_DRDY_GPIO_Port GPIOE
#define IIS2MDC_DRDY_Pin GPIO_PIN_0
#define IIS2MDC_DRDY_GPIO_Port GPIOE
#define IIS2MDC_DRDY_EXTI_IRQn EXTI0_IRQn
#define BMI088_MOSI_Pin GPIO_PIN_7
#define BMI088_MOSI_GPIO_Port GPIOD
#define J103_1_TELEM_UART2_RTS_Pin GPIO_PIN_4
#define J103_1_TELEM_UART2_RTS_GPIO_Port GPIOD
#define J105_4_CAN1_RX_Pin GPIO_PIN_0
#define J105_4_CAN1_RX_GPIO_Port GPIOD
#define J106_PWM01_Pin GPIO_PIN_8
#define J106_PWM01_GPIO_Port GPIOA
#define J106_PWM03_Pin GPIO_PIN_10
#define J106_PWM03_GPIO_Port GPIOA
#define MAG_MISO_Pin GPIO_PIN_2
#define MAG_MISO_GPIO_Port GPIOC
#define ADIS165XX_MOSI_Pin GPIO_PIN_6
#define ADIS165XX_MOSI_GPIO_Port GPIOE
#define J105_4_CAN1_TX_Pin GPIO_PIN_1
#define J105_4_CAN1_TX_GPIO_Port GPIOD
#define J102_SDMMC1_D1_Pin GPIO_PIN_9
#define J102_SDMMC1_D1_GPIO_Port GPIOC
#define TP6_Pin GPIO_PIN_7
#define TP6_GPIO_Port GPIOC
#define ADC_5V0_Pin GPIO_PIN_0
#define ADC_5V0_GPIO_Port GPIOC
#define ADC_IBATT_Pin GPIO_PIN_1
#define ADC_IBATT_GPIO_Port GPIOC
#define MAG_MOSI_Pin GPIO_PIN_3
#define MAG_MOSI_GPIO_Port GPIOC
#define J102_SDMMC1_D0_Pin GPIO_PIN_8
#define J102_SDMMC1_D0_GPIO_Port GPIOC
#define J106_PWM09_Pin GPIO_PIN_6
#define J106_PWM09_GPIO_Port GPIOC
#define ADC_VBAT_Pin GPIO_PIN_0
#define ADC_VBAT_GPIO_Port GPIOA
#define BMI088_ACCEL_CSn_Pin GPIO_PIN_4
#define BMI088_ACCEL_CSn_GPIO_Port GPIOA
#define ADC_3V3_Pin GPIO_PIN_4
#define ADC_3V3_GPIO_Port GPIOC
#define BARO_MOSI_Pin GPIO_PIN_2
#define BARO_MOSI_GPIO_Port GPIOB
#define J105_2_DRDY_Pin GPIO_PIN_10
#define J105_2_DRDY_GPIO_Port GPIOE
#define J105_2_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define J106_PWM04_Pin GPIO_PIN_14
#define J106_PWM04_GPIO_Port GPIOE
#define J106_PWM08_Pin GPIO_PIN_15
#define J106_PWM08_GPIO_Port GPIOD
#define DPS310_DRDY_Pin GPIO_PIN_11
#define DPS310_DRDY_GPIO_Port GPIOD
#define DPS310_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define J105_2_SPI4_EXT_CLK_Pin GPIO_PIN_15
#define J105_2_SPI4_EXT_CLK_GPIO_Port GPIOB
#define BMI088_GYRO_DRDY_Pin GPIO_PIN_1
#define BMI088_GYRO_DRDY_GPIO_Port GPIOA
#define BMI088_GYRO_DRDY_EXTI_IRQn EXTI1_IRQn
#define BMI088_SCK_Pin GPIO_PIN_5
#define BMI088_SCK_GPIO_Port GPIOA
#define ADC_12V_Pin GPIO_PIN_5
#define ADC_12V_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_7
#define LED_RED_GPIO_Port GPIOE
#define J106_PWM02_Pin GPIO_PIN_11
#define J106_PWM02_GPIO_Port GPIOE
#define LED_GRN_Pin GPIO_PIN_15
#define LED_GRN_GPIO_Port GPIOE
#define J106_PWM05_Pin GPIO_PIN_14
#define J106_PWM05_GPIO_Port GPIOD
#define BMI088_GYRO_CSn_Pin GPIO_PIN_10
#define BMI088_GYRO_CSn_GPIO_Port GPIOD
#define ADIS165XX_EXT_CLK_Pin GPIO_PIN_14
#define ADIS165XX_EXT_CLK_GPIO_Port GPIOB
#define J105_3_SYNC_IN_Pin GPIO_PIN_2
#define J105_3_SYNC_IN_GPIO_Port GPIOA
#define J105_3_SYNC_IN_EXTI_IRQn EXTI2_IRQn
#define BMI088_MISO_Pin GPIO_PIN_6
#define BMI088_MISO_GPIO_Port GPIOA
#define TP5_Pin GPIO_PIN_0
#define TP5_GPIO_Port GPIOB
#define LED_BLU_Pin GPIO_PIN_8
#define LED_BLU_GPIO_Port GPIOE
#define ADIS165XX_RESET_Pin GPIO_PIN_12
#define ADIS165XX_RESET_GPIO_Port GPIOE
#define J105_1_I2C2_SCL_Pin GPIO_PIN_10
#define J105_1_I2C2_SCL_GPIO_Port GPIOB
#define MAG_SCK_Pin GPIO_PIN_13
#define MAG_SCK_GPIO_Port GPIOB
#define J103_2_RC_UART3_RX_Pin GPIO_PIN_9
#define J103_2_RC_UART3_RX_GPIO_Port GPIOD
#define J106_PWM06_Pin GPIO_PIN_13
#define J106_PWM06_GPIO_Port GPIOD
#define GPS_1PPS_Pin GPIO_PIN_3
#define GPS_1PPS_GPIO_Port GPIOA
#define GPS_1PPS_EXTI_IRQn EXTI3_IRQn
#define ADC_VSERVO_Pin GPIO_PIN_7
#define ADC_VSERVO_GPIO_Port GPIOA
#define J105_3_SYNC_OUT_Pin GPIO_PIN_1
#define J105_3_SYNC_OUT_GPIO_Port GPIOB
#define PITOT_DRDY_Pin GPIO_PIN_9
#define PITOT_DRDY_GPIO_Port GPIOE
#define PITOT_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define ADIS165XX_MISO_Pin GPIO_PIN_13
#define ADIS165XX_MISO_GPIO_Port GPIOE
#define J105_1_I2C2_SDA_Pin GPIO_PIN_11
#define J105_1_I2C2_SDA_GPIO_Port GPIOB
#define IIS2MDC_CSn_Pin GPIO_PIN_12
#define IIS2MDC_CSn_GPIO_Port GPIOB
#define J103_2_RC_UART3_TX_Pin GPIO_PIN_8
#define J103_2_RC_UART3_TX_GPIO_Port GPIOD
#define J106_PWM07_Pin GPIO_PIN_12
#define J106_PWM07_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
