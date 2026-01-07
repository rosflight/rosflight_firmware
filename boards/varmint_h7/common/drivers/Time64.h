/**
 ******************************************************************************
 * File     : Time.h
 * Date     : Sep 20, 2023
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

#ifndef TIME64_H_
#define TIME64_H_

#include "CommonConfig.h"
#include "Status.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

class Time64 : public Status
{
public:
  uint32_t init(TIM_HandleTypeDef * htim_low, TIM_TypeDef * instance_low, TIM_HandleTypeDef * htim_high,
                TIM_TypeDef * instance_high)
  {
    snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Time64");
    initializationStatus_ = DRIVER_OK;

    htimLow_ = htim_low;
    htimHigh_ = htim_high;

    if (instance_low == TIM5 || instance_low == TIM2) shift_ = 32;
    else shift_ = 16;

    // low timer (master)
    {
      TIM_ClockConfigTypeDef sClockSourceConfig = {0};
      TIM_MasterConfigTypeDef sMasterConfig = {0};
      htimLow_->Instance = instance_low;
      htimLow_->Init.Prescaler = 199;
      htimLow_->Init.CounterMode = TIM_COUNTERMODE_UP;
      if (htimLow_->Instance == TIM5 || htimLow_->Instance == TIM2) htimLow_->Init.Period = 0xFFFFFFFF; // 32-bit timer
      else htimLow_->Init.Period = 0xFFFF;                                                              // 16-bit timer
      htimLow_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      if (htimLow_->Instance == TIM8) htimLow_->Init.RepetitionCounter = 0;
      htimLow_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
      if (HAL_TIM_Base_Init(htimLow_) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return initializationStatus_;
      }

      sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
      if (HAL_TIM_ConfigClockSource(htimLow_, &sClockSourceConfig) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return initializationStatus_;
      }
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
      if (htimLow_->Instance == TIM8) sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;

      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(htimLow_, &sMasterConfig) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return initializationStatus_;
      }
    }
    // high timer (slave)
    {
      TIM_SlaveConfigTypeDef sSlaveConfig = {0};
      TIM_MasterConfigTypeDef sMasterConfig = {0};
      htimHigh_->Instance = instance_high;

      htimHigh_->Init.Prescaler = 0;
      htimHigh_->Init.CounterMode = TIM_COUNTERMODE_UP;
      if (htimHigh_->Instance == TIM5 || htimHigh_->Instance == TIM2)
        htimHigh_->Init.Period = 0xFFFFFFFF; // 32-bit timer
      else htimHigh_->Init.Period = 0xFFFF;  // 16-bit timer
      htimHigh_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      if (htimHigh_->Instance == TIM8) htimHigh_->Init.RepetitionCounter = 0;
      htimHigh_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
      if (HAL_TIM_Base_Init(htimHigh_) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return initializationStatus_;
      }

      sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
      // All valid combinations have at least one 32-bit timer (TIM2 or 5)
      if ((htimHigh_->Instance == TIM5 || htimHigh_->Instance == TIM2) && htimLow_->Instance == TIM1)
        sSlaveConfig.InputTrigger = TIM_TS_ITR0;
      else if ((htimHigh_->Instance == TIM5 || htimHigh_->Instance == TIM2) && htimLow_->Instance == TIM3)
        sSlaveConfig.InputTrigger = TIM_TS_ITR2;
      else if ((htimHigh_->Instance == TIM5 || htimHigh_->Instance == TIM2) && htimLow_->Instance == TIM4)
        sSlaveConfig.InputTrigger = TIM_TS_ITR3;
      else if ((htimHigh_->Instance == TIM5 || htimHigh_->Instance == TIM2) && htimLow_->Instance == TIM8)
        sSlaveConfig.InputTrigger = TIM_TS_ITR1;
      else if (htimHigh_->Instance == TIM3 && htimLow_->Instance == TIM2) sSlaveConfig.InputTrigger = TIM_TS_ITR1;
      else if (htimHigh_->Instance == TIM4 && htimLow_->Instance == TIM2) sSlaveConfig.InputTrigger = TIM_TS_ITR1;
      else if (htimHigh_->Instance == TIM8 && htimLow_->Instance == TIM2) sSlaveConfig.InputTrigger = TIM_TS_ITR1;
      else if (htimHigh_->Instance == TIM8 && htimLow_->Instance == TIM5) sSlaveConfig.InputTrigger = TIM_TS_ITR3;
      else if (htimHigh_->Instance == TIM12 && htimLow_->Instance == TIM5) sSlaveConfig.InputTrigger = TIM_TS_ITR1;
      else {
        initializationStatus_ |= TIMERS_INVALID;
        return initializationStatus_;
      }

      if (HAL_TIM_SlaveConfigSynchro(htimHigh_, &sSlaveConfig) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return DRIVER_HAL_ERROR;
      }
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      if (htimHigh_->Instance == TIM8) sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(htimHigh_, &sMasterConfig) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return initializationStatus_;
      }
    }

    // Note priority is set in HAL_TIM_Base_MspInit().

    __HAL_TIM_SET_COUNTER(htimHigh_, 0);
    __HAL_TIM_SET_COUNTER(htimLow_, 0);
    HAL_TIM_Base_Start_IT(htimHigh_); // Startup our overflow counter.
    HAL_TIM_Base_Start_IT(htimLow_);  // Startup our us counter.
    return initializationStatus_;
  }

  //    inline uint32_t UsLow(void)
  //    {
  //        return __HAL_TIM_GET_COUNTER(htimLow_);
  //    }

  uint64_t Us(void)
  {
    volatile uint32_t low1 = __HAL_TIM_GET_COUNTER(htimLow_); // htimLow_->Instance->CNT;
    volatile uint32_t high1 = __HAL_TIM_GET_COUNTER(htimHigh_);
    volatile uint32_t low2 = __HAL_TIM_GET_COUNTER(htimLow_);
    volatile uint32_t high2 = __HAL_TIM_GET_COUNTER(htimHigh_);
    if ((low1 > low2) && (high1 == high2)) {high1 = high1-1;} // rollover correction
    uint64_t us = ((high1 << shift_) | low1)&0x0000FFFFFFFFFFFF;
    return us;
  }

  void dUs(uint32_t dt)
  {
    uint64_t endtimestamp = Us() + dt;
    while (Us() < endtimestamp) {};
  }

  void dMs(uint32_t dt)
  {
    uint64_t endtimestamp = Us() + 1000LLU * dt;
    while (Us() < endtimestamp) {};
  }

private:
  TIM_HandleTypeDef * htimLow_;
  TIM_HandleTypeDef * htimHigh_;
  uint32_t shift_;
};

#endif /* TIME64_H_ */
