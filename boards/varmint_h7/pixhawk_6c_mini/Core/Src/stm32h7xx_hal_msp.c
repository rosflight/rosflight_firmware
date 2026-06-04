#include "main.h"

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef * htim_pwm)
{
  if (htim_pwm->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  } else if (htim_pwm->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
  } else if (htim_pwm->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef * htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM1) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = FMU_CH1_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(FMU_CH1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FMU_CH2_Pin | FMU_CH3_Pin | FMU_CH4_Pin;
    HAL_GPIO_Init(FMU_CH2_GPIO_Port, &GPIO_InitStruct);
  } else if (htim->Instance == TIM4) {
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = FMU_CH5_Pin | FMU_CH6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  } else if (htim->Instance == TIM5) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = FMU_CH7_Pin | FMU_CH8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef * htim_pwm)
{
  if (htim_pwm->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_GPIO_DeInit(FMU_CH1_GPIO_Port, FMU_CH1_Pin);
    HAL_GPIO_DeInit(FMU_CH2_GPIO_Port, FMU_CH2_Pin | FMU_CH3_Pin | FMU_CH4_Pin);
  } else if (htim_pwm->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, FMU_CH5_Pin | FMU_CH6_Pin);
  } else if (htim_pwm->Instance == TIM5) {
    __HAL_RCC_TIM5_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, FMU_CH7_Pin | FMU_CH8_Pin);
  }
}
