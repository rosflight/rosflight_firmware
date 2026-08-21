
/**
 ******************************************************************************
 * File     : Polling.cpph
 *
 * Date     : June 20, 2024
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
#include "Polling.h"
#include "BoardConfig.h"
#include "stdint.h"

uint32_t PollingTimer::init(TIM_HandleTypeDef * htim, TIM_TypeDef * instance, uint32_t channel, uint32_t polling_period_us)
{
  initializationStatus_ = DRIVER_OK;

  if (polling_period_us == 0U) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  instance_ = instance;
  polling_period_us_ = polling_period_us;
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim->Instance = instance_;
  htim->Init.Prescaler = 199;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = polling_period_us_ - 1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(htim) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }

  HAL_TIM_PWM_Start(htim, channel); // (10kHz) to service polling routines
  HAL_TIM_Base_Start_IT(htim);
  return initializationStatus_;
}

bool PollingTimer::is_my(TIM_HandleTypeDef * htim) const
{
  return (instance_ != nullptr) && (htim->Instance == instance_);
}

bool PollingTimer::polling_state(uint64_t poll_counter, uint32_t rollover_us, uint16_t & state) const
{
  if (polling_period_us_ == 0U) return false;

  const uint32_t rollover_counts = rollover_us / polling_period_us_;
  if (rollover_counts == 0U) return false;

  state = (uint16_t) (poll_counter % rollover_counts);
  return true;
}
