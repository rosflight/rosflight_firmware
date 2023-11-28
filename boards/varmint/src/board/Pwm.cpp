/**
******************************************************************************
* File     : Pwm.cpp
* Date     : Nov 3, 2023
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

#include <Driver.h>
#include <BoardConfig.h>

uint32_t init_pwm_timers(uint32_t servo_pwm_period_us)
{
	{
		TIM_MasterConfigTypeDef sMasterConfig = {0};
		TIM_OC_InitTypeDef sConfigOC = {0};
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

		htim1.Instance = TIM1;
		htim1.Init.Prescaler = (SERVO_PWM_CLK_DIV);
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = servo_pwm_period_us;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.RepetitionCounter = 0;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) return DRIVER_HAL_ERROR;
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) return DRIVER_HAL_ERROR;
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = (SERVO_PWM_CENTER);
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) return DRIVER_HAL_ERROR;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) return DRIVER_HAL_ERROR;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) return DRIVER_HAL_ERROR;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) return DRIVER_HAL_ERROR;
		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.BreakFilter = 0;
		sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
		sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
		sBreakDeadTimeConfig.Break2Filter = 0;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) return DRIVER_HAL_ERROR;
		HAL_TIM_MspPostInit(&htim1);
	}
	{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (SERVO_PWM_CLK_DIV);
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = servo_pwm_period_us;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) return DRIVER_HAL_ERROR;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) return DRIVER_HAL_ERROR;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (SERVO_PWM_CENTER);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) return DRIVER_HAL_ERROR;
    HAL_TIM_MspPostInit(&htim3);
  }
  {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (SERVO_PWM_CLK_DIV);
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = servo_pwm_period_us;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) return DRIVER_HAL_ERROR;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) return DRIVER_HAL_ERROR;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (SERVO_PWM_CENTER);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) return DRIVER_HAL_ERROR;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) return DRIVER_HAL_ERROR;
    HAL_TIM_MspPostInit(&htim4);
  }
  return DRIVER_OK;
}
