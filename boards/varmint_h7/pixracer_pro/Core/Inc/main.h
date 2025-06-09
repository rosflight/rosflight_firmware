/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
void MX_FDCAN2_Init(void);
void MX_I2C1_Init(void);
void MX_SDMMC1_SD_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_SPI5_Init(void);
void MX_SPI6_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_UART4_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void MX_RNG_Init(void);
void MX_RTC_Init(void);
void MX_TIM5_Init(void);
void MX_TIM7_Init(void);
void MX_FDCAN1_Init(void);
void MX_ADC3_Init(void);
void MX_ADC1_Init(void);
void MX_CRC_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);
void MX_TIM12_Init(void);
void MX_TIM3_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VDD_3V3_SENSORS_EN_Pin GPIO_PIN_3
#define VDD_3V3_SENSORS_EN_GPIO_Port GPIOE
#define RC_UART8_TX_Pin GPIO_PIN_1
#define RC_UART8_TX_GPIO_Port GPIOE
#define RC_UART8_RX_Pin GPIO_PIN_0
#define RC_UART8_RX_GPIO_Port GPIOE
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define VDD_BRICK_VALID_Pin GPIO_PIN_5
#define VDD_BRICK_VALID_GPIO_Port GPIOB
#define EXTERNAL_SPI6_MOSI_Pin GPIO_PIN_14
#define EXTERNAL_SPI6_MOSI_GPIO_Port GPIOG
#define EXTERNAL_SPI6_SCK_Pin GPIO_PIN_13
#define EXTERNAL_SPI6_SCK_GPIO_Port GPIOG
#define LED_BLUE_Pin GPIO_PIN_3
#define LED_BLUE_GPIO_Port GPIOB
#define DPS310_CSn_Pin GPIO_PIN_7
#define DPS310_CSn_GPIO_Port GPIOD
#define SDMMC_CK_Pin GPIO_PIN_12
#define SDMMC_CK_GPIO_Port GPIOC
#define BUZZER_PWM_Pin GPIO_PIN_15
#define BUZZER_PWM_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define SPEKTRUM_POWER_EN_Pin GPIO_PIN_4
#define SPEKTRUM_POWER_EN_GPIO_Port GPIOE
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define FMU_UART1_RX_Pin GPIO_PIN_7
#define FMU_UART1_RX_GPIO_Port GPIOB
#define FMU_UART1_TX_Pin GPIO_PIN_6
#define FMU_UART1_TX_GPIO_Port GPIOB
#define EXTERNAL_SPI6_MISO_Pin GPIO_PIN_12
#define EXTERNAL_SPI6_MISO_GPIO_Port GPIOG
#define TELEM1_USART2_RX_Pin GPIO_PIN_6
#define TELEM1_USART2_RX_GPIO_Port GPIOD
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define SDMMC_D3_Pin GPIO_PIN_11
#define SDMMC_D3_GPIO_Port GPIOC
#define SDMMC_D2_Pin GPIO_PIN_10
#define SDMMC_D2_GPIO_Port GPIOC
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define PWM_08_Pin GPIO_PIN_6
#define PWM_08_GPIO_Port GPIOI
#define PWM_07_Pin GPIO_PIN_5
#define PWM_07_GPIO_Port GPIOI
#define EXTERNAL_SPI6_CS_Pin GPIO_PIN_9
#define EXTERNAL_SPI6_CS_GPIO_Port GPIOG
#define TELEM1_USART2_TX_Pin GPIO_PIN_5
#define TELEM1_USART2_TX_GPIO_Port GPIOD
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define VDD_1V8_SENSORS_EN_Pin GPIO_PIN_3
#define VDD_1V8_SENSORS_EN_GPIO_Port GPIOI
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define TELEM1_USART2_RTS_Pin GPIO_PIN_4
#define TELEM1_USART2_RTS_GPIO_Port GPIOD
#define TELEM1_USART2_CTS_Pin GPIO_PIN_3
#define TELEM1_USART2_CTS_GPIO_Port GPIOD
#define SDMMC_CMD_Pin GPIO_PIN_2
#define SDMMC_CMD_GPIO_Port GPIOD
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define SDMMC_D1_Pin GPIO_PIN_9
#define SDMMC_D1_GPIO_Port GPIOC
#define CRYSTAL_24MHz_IN_Pin GPIO_PIN_0
#define CRYSTAL_24MHz_IN_GPIO_Port GPIOH
#define SDMMC_D0_Pin GPIO_PIN_8
#define SDMMC_D0_GPIO_Port GPIOC
#define RC_USART6_RX_Pin GPIO_PIN_7
#define RC_USART6_RX_GPIO_Port GPIOC
#define CRYSTAL_24MHz_OUT_Pin GPIO_PIN_1
#define CRYSTAL_24MHz_OUT_GPIO_Port GPIOH
#define BMI088_INT2_ACCEL_Pin GPIO_PIN_2
#define BMI088_INT2_ACCEL_GPIO_Port GPIOF
#define BMI088_INT1_ACCEL_Pin GPIO_PIN_1
#define BMI088_INT1_ACCEL_GPIO_Port GPIOF
#define BMI088_INT1_ACCEL_EXTI_IRQn EXTI1_IRQn
#define RC_USART6_TX_NC_Pin GPIO_PIN_6
#define RC_USART6_TX_NC_GPIO_Port GPIOC
#define BMI088_INT3_GYRO_Pin GPIO_PIN_3
#define BMI088_INT3_GYRO_GPIO_Port GPIOF
#define BMI088_INT3_GYRO_EXTI_IRQn EXTI3_IRQn
#define BMI088_INT4_GYRO_Pin GPIO_PIN_4
#define BMI088_INT4_GYRO_GPIO_Port GPIOF
#define BMI088_INT4_GYRO_EXTI_IRQn EXTI4_IRQn
#define PWM_VOLT_SEL_Pin GPIO_PIN_6
#define PWM_VOLT_SEL_GPIO_Port GPIOG
#define BMI088_SCK_Pin GPIO_PIN_7
#define BMI088_SCK_GPIO_Port GPIOF
#define BMI088_ACCEL_CS_Pin GPIO_PIN_6
#define BMI088_ACCEL_CS_GPIO_Port GPIOF
#define BMI088_GYRO_CS_Pin GPIO_PIN_10
#define BMI088_GYRO_CS_GPIO_Port GPIOF
#define BMI088_MOSI_Pin GPIO_PIN_9
#define BMI088_MOSI_GPIO_Port GPIOF
#define BMI088_MISO_Pin GPIO_PIN_8
#define BMI088_MISO_GPIO_Port GPIOF
#define ICM_20602_DRDY_Pin GPIO_PIN_15
#define ICM_20602_DRDY_GPIO_Port GPIOD
#define VBUS_VALID_Pin GPIO_PIN_0
#define VBUS_VALID_GPIO_Port GPIOC
#define ADC_RSSI_V_Pin GPIO_PIN_1
#define ADC_RSSI_V_GPIO_Port GPIOC
#define ICM_20602_CS_Pin GPIO_PIN_2
#define ICM_20602_CS_GPIO_Port GPIOC
#define PWM_06_Pin GPIO_PIN_14
#define PWM_06_GPIO_Port GPIOD
#define PWM_05_Pin GPIO_PIN_13
#define PWM_05_GPIO_Port GPIOD
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define ADC_5V_V_Pin GPIO_PIN_4
#define ADC_5V_V_GPIO_Port GPIOA
#define PWM_02_Pin GPIO_PIN_13
#define PWM_02_GPIO_Port GPIOE
#define TELEM2_USART3_RTS_Pin GPIO_PIN_12
#define TELEM2_USART3_RTS_GPIO_Port GPIOD
#define TELEM2_USART3_CTS_Pin GPIO_PIN_11
#define TELEM2_USART3_CTS_GPIO_Port GPIOD
#define FRAM_CS_Pin GPIO_PIN_10
#define FRAM_CS_GPIO_Port GPIOD
#define ADC_BATT_V_Pin GPIO_PIN_2
#define ADC_BATT_V_GPIO_Port GPIOA
#define ICM_MISO_Pin GPIO_PIN_6
#define ICM_MISO_GPIO_Port GPIOA
#define ICM_SCK_Pin GPIO_PIN_5
#define ICM_SCK_GPIO_Port GPIOA
#define DEBUG_UART7_TX_Pin GPIO_PIN_8
#define DEBUG_UART7_TX_GPIO_Port GPIOE
#define PWM_04_Pin GPIO_PIN_9
#define PWM_04_GPIO_Port GPIOE
#define PWM_03_Pin GPIO_PIN_11
#define PWM_03_GPIO_Port GPIOE
#define PWM_01_Pin GPIO_PIN_14
#define PWM_01_GPIO_Port GPIOE
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define TELEM2_USART3_RX_Pin GPIO_PIN_9
#define TELEM2_USART3_RX_GPIO_Port GPIOD
#define TELEM2_USART3_TX_Pin GPIO_PIN_8
#define TELEM2_USART3_TX_GPIO_Port GPIOD
#define ADC_BATT_I_Pin GPIO_PIN_3
#define ADC_BATT_I_GPIO_Port GPIOA
#define ICM_MOSI_Pin GPIO_PIN_7
#define ICM_MOSI_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOB
#define RC_TIM_INPUT_Pin GPIO_PIN_0
#define RC_TIM_INPUT_GPIO_Port GPIOB
#define CAN1_EN_Pin GPIO_PIN_11
#define CAN1_EN_GPIO_Port GPIOF
#define CAN2_EN_Pin GPIO_PIN_14
#define CAN2_EN_GPIO_Port GPIOF
#define DEBUG_UART7_RX_Pin GPIO_PIN_7
#define DEBUG_UART7_RX_GPIO_Port GPIOE
#define ICM_20948_DRDY_Pin GPIO_PIN_12
#define ICM_20948_DRDY_GPIO_Port GPIOE
#define ICM_20948_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define ICM_20948_CS_Pin GPIO_PIN_15
#define ICM_20948_CS_GPIO_Port GPIOE
#define DPS310_FRAM_SCK_Pin GPIO_PIN_10
#define DPS310_FRAM_SCK_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOB
#define DPS310_FRAM_MISO_Pin GPIO_PIN_14
#define DPS310_FRAM_MISO_GPIO_Port GPIOB
#define DPS310_FRAM_MOSI_Pin GPIO_PIN_15
#define DPS310_FRAM_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
