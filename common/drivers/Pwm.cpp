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

#include <BoardConfig.h>
#include <Driver.h>
#include <Pwm.h>

// Notes on DSHOT
// DSHOT  Bitrate	T1H      T0H    Bit(µs)	Frame (µs)
//  150	  150kbit/s	5.00	2.50	6.67	106.72
//  300	  300kbit/s	2.50	1.25	3.33	53.28 < we use this
//  600	  600kbit/s	1.25	0.625	1.67	26.72
// 1200	 1200kbit/s	0.625	0.313	0.83	13.28

DATA_RAM PwmBlockStructure pwm_init[PWM_TIMER_BLOCKS] = PWM_INIT_DEFINE;

DMA_RAM uint32_t pwm_dma_buf[PWM_TIMER_BLOCKS][PWM_DMA_BUFFER_LEN];

void Pwm::updateConfig(const float * rate, uint32_t channels)
{
  channels = (channels < PWM_CHANNELS) ? channels : PWM_CHANNELS;

  for (uint32_t ch = 0; ch < channels; ch++) HAL_TIM_PWM_Stop(htim_[ch], chan_[ch]);

  for (uint32_t ch = 0; ch < channels; ch++) setRate(ch, rate[ch]);

  for (uint32_t ch = 0; ch < channels; ch++) HAL_TIM_PWM_Start(htim_[ch], chan_[ch]);
}

uint32_t Pwm::init(void)
{
  block_ = pwm_init;
  dmaBuf_ = pwm_dma_buf;

  // Clear lookup table
  for (uint32_t output_index = 0; output_index < PWM_CHANNELS; output_index++) {
    htim_[output_index] = nullptr;
    chan_[output_index] = PWM_CHAN_IGNORE;
    blockIndex_[output_index] = (uint32_t) (-1);
  }

  // Fill-in lookup tables
  for (uint32_t bk = 0; bk < PWM_TIMER_BLOCKS; bk++) {
    for (uint32_t ch = 0; ch < 4; ch++) {
      uint32_t output_index = block_[bk].chan[ch];
      if (output_index < PWM_CHANNELS) {
        blockIndex_[output_index] = bk;
        htim_[output_index] = block_[bk].htim;
        chan_[output_index] = (uint32_t) ch << 2; // Note: TIM_CHANNEL_x = (x-1)*4; up to x==4
      }
    }
  }

  for (uint32_t bk = 0; bk < PWM_TIMER_BLOCKS; bk++) {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    TIM_HandleTypeDef * htim = block_[bk].htim;

    if (htim == &htim1) htim->Instance = TIM1;
    else if (htim == &htim3) htim->Instance = TIM3;
    else if (htim == &htim4) htim->Instance = TIM4;
    else if (htim == &htim8) htim->Instance = TIM8;
    else return DRIVER_HAL_ERROR;

    if ((block_[bk].rate >= 150000) && (block_[bk].type == PWM_DSHOT)) // DSHOT
    {
      htim->Init.Prescaler = 0;
      htim->Init.Period = (uint64_t) 200000000 / block_[bk].rate - 1;
    } else if ((block_[bk].type == PWM_STANDARD) && (block_[bk].rate < 490)) {
      htim->Init.Prescaler = 199;
      htim->Init.Period = (uint64_t) 1000000 / block_[bk].rate - 1;
    } else return DRIVER_HAL_ERROR;

    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(htim) != HAL_OK) return DRIVER_HAL_ERROR;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) return DRIVER_HAL_ERROR;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    //        sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    for (uint32_t ch = 0; ch < 4; ch++) {
      if (block_[bk].chan[ch] < PWM_CHANNELS) {
        sConfigOC.Pulse = 0; // default to flat line output
        if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, (uint32_t) ch * 4) != HAL_OK) return DRIVER_HAL_ERROR;
      }
    }

    if ((htim == &htim1) || (htim == &htim8)) {
      TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
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
      if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK) return DRIVER_HAL_ERROR;
    }
    HAL_TIM_MspPostInit(htim);
  }

  return DRIVER_OK;
}
