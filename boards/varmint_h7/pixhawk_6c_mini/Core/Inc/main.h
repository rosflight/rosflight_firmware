#ifndef PIXHAWK_6C_MINI_MAIN_H
#define PIXHAWK_6C_MINI_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

#define FMU_LED_RED_Pin GPIO_PIN_10
#define FMU_LED_RED_GPIO_Port GPIOD
#define FMU_LED_BLUE_Pin GPIO_PIN_11
#define FMU_LED_BLUE_GPIO_Port GPIOD
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA

void Error_Handler(void);
void SystemClock_Config(void);
void MX_USB_OTG_FS_PCD_Init(void);

#ifdef __cplusplus
}
#endif

#endif
