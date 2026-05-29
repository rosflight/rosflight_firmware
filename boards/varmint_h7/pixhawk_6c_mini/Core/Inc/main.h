#ifndef PIXHAWK_6C_MINI_MAIN_H
#define PIXHAWK_6C_MINI_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern SD_HandleTypeDef hsd2;
extern CRC_HandleTypeDef hcrc;

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
#define SDMMC2_CK_Pin GPIO_PIN_6
#define SDMMC2_CK_GPIO_Port GPIOD
#define SDMMC2_CMD_Pin GPIO_PIN_7
#define SDMMC2_CMD_GPIO_Port GPIOD

void Error_Handler(void);
void SystemClock_Config(void);
void MX_CRC_Init(void);
void MX_GPIO_Init(void);
HAL_StatusTypeDef MX_SDMMC2_SD_Init(void);
void MX_USB_OTG_FS_PCD_Init(void);

#ifdef __cplusplus
}
#endif

#endif
