/**
 ******************************************************************************
 * File     : Polling.h
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

#ifndef DRIVERS_POLLING_H_
#define DRIVERS_POLLING_H_

//#include "stm32h7xx_hal.h"

#include "CommonConfig.h"
#include "stdint.h"

typedef uint16_t PollingState;

class Polling: public Status {
public:
  uint32_t init(
    TIM_HandleTypeDef * htim,
    TIM_TypeDef * instance,
    uint32_t channel,
    uint32_t period_us
  )
  {
    snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Polling");
    initializationStatus_ = DRIVER_OK;

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    period_us_ = period_us;
    htim_ = htim;

    htim->Instance = instance;
    htim->Init.Prescaler = 199;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = period_us - 1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK) return initializationStatus_ = DRIVER_HAL_ERROR;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) return initializationStatus_ = DRIVER_HAL_ERROR;

    HAL_TIM_PWM_Start(htim, channel); // (10kHz) to service polling routines
    HAL_TIM_Base_Start_IT(htim);

    return initializationStatus_;
  }

  bool isMy(TIM_HandleTypeDef * htim) { return htim == htim_; }

    uint32_t index(uint64_t counter, uint64_t dt_us) {
    return  (uint32_t)(counter % (dt_us / period_us_));
  }

  uint32_t period_us(void) {return period_us_;}


private:
  TIM_HandleTypeDef * htim_;
  uint64_t period_us_;

};




#endif /* DRIVERS_POLLING_H_ */
