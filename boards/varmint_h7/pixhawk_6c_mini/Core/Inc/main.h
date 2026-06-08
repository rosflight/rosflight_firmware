#ifndef PIXHAWK_6C_MINI_MAIN_H
#define PIXHAWK_6C_MINI_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SD_HandleTypeDef hsd2;
extern CRC_HandleTypeDef hcrc;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

#define FMU_LED_RED_Pin GPIO_PIN_10
#define FMU_LED_RED_GPIO_Port GPIOD
#define FMU_LED_BLUE_Pin GPIO_PIN_11
#define FMU_LED_BLUE_GPIO_Port GPIOD
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SDMMC2_D2_Pin GPIO_PIN_3
#define SDMMC2_D2_GPIO_Port GPIOB
#define SDMMC2_D3_Pin GPIO_PIN_4
#define SDMMC2_D3_GPIO_Port GPIOB
#define SDMMC2_D0_Pin GPIO_PIN_14
#define SDMMC2_D0_GPIO_Port GPIOB
#define SDMMC2_D1_Pin GPIO_PIN_15
#define SDMMC2_D1_GPIO_Port GPIOB
#define VDD_3V3_SENSORS_EN_Pin GPIO_PIN_2
#define VDD_3V3_SENSORS_EN_GPIO_Port GPIOB
#define N_VDD_5V_PERIPH_EN_Pin GPIO_PIN_2
#define N_VDD_5V_PERIPH_EN_GPIO_Port GPIOE
#define SDMMC2_CK_Pin GPIO_PIN_6
#define SDMMC2_CK_GPIO_Port GPIOD
#define SDMMC2_CMD_Pin GPIO_PIN_7
#define SDMMC2_CMD_GPIO_Port GPIOD
#define FMU_BAT1_I_Pin GPIO_PIN_4
#define FMU_BAT1_I_GPIO_Port GPIOC
#define FMU_BAT1_V_Pin GPIO_PIN_5
#define FMU_BAT1_V_GPIO_Port GPIOC
#define IMU_SPI1_SCK_Pin GPIO_PIN_5
#define IMU_SPI1_SCK_GPIO_Port GPIOA
#define IMU_SPI1_MISO_Pin GPIO_PIN_6
#define IMU_SPI1_MISO_GPIO_Port GPIOA
#define IMU_SPI1_MOSI_Pin GPIO_PIN_7
#define IMU_SPI1_MOSI_GPIO_Port GPIOA
#define IMU_ICM42688_CS_Pin GPIO_PIN_13
#define IMU_ICM42688_CS_GPIO_Port GPIOC
#define IMU_ICM42688_DRDY_Pin GPIO_PIN_6
#define IMU_ICM42688_DRDY_GPIO_Port GPIOE
#define MAG_I2C4_SCL_Pin GPIO_PIN_12
#define MAG_I2C4_SCL_GPIO_Port GPIOD
#define MAG_I2C4_SDA_Pin GPIO_PIN_13
#define MAG_I2C4_SDA_GPIO_Port GPIOD
#define GPS1_UART_RX_Pin GPIO_PIN_10
#define GPS1_UART_RX_GPIO_Port GPIOA
#define GPS1_UART_TX_Pin GPIO_PIN_6
#define GPS1_UART_TX_GPIO_Port GPIOB
#define TELEM1_UART7_RX_Pin GPIO_PIN_7
#define TELEM1_UART7_RX_GPIO_Port GPIOE
#define TELEM1_UART7_TX_Pin GPIO_PIN_8
#define TELEM1_UART7_TX_GPIO_Port GPIOE
#define GPS1_I2C1_SDA_Pin GPIO_PIN_7
#define GPS1_I2C1_SDA_GPIO_Port GPIOB
#define GPS1_I2C1_SCL_Pin GPIO_PIN_8
#define GPS1_I2C1_SCL_GPIO_Port GPIOB

#define FMU_CH1_Pin GPIO_PIN_8
#define FMU_CH1_GPIO_Port GPIOA
#define FMU_CH2_Pin GPIO_PIN_11
#define FMU_CH2_GPIO_Port GPIOE
#define FMU_CH3_Pin GPIO_PIN_13
#define FMU_CH3_GPIO_Port GPIOE
#define FMU_CH4_Pin GPIO_PIN_14
#define FMU_CH4_GPIO_Port GPIOE
#define FMU_CH5_Pin GPIO_PIN_14
#define FMU_CH5_GPIO_Port GPIOD
#define FMU_CH6_Pin GPIO_PIN_15
#define FMU_CH6_GPIO_Port GPIOD
#define FMU_CH7_Pin GPIO_PIN_0
#define FMU_CH7_GPIO_Port GPIOA
#define FMU_CH8_Pin GPIO_PIN_1
#define FMU_CH8_GPIO_Port GPIOA

void Error_Handler(void);
void SystemClock_Config(void);
void MX_DMA_Init(void);
void MX_CRC_Init(void);
void MX_ADC1_Init(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C4_Init(void);
HAL_StatusTypeDef MX_SDMMC2_SD_Init(void);
void MX_SPI1_Init(void);
void MX_USART1_UART_Init(void);
void MX_UART7_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef * htim);

#ifdef __cplusplus
}
#endif

#endif
