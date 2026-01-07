/**
 ******************************************************************************
 * File     : Pwm.h
 * Date     : Oct 5, 2023
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

#ifndef PWM_H_
#define PWM_H_

#include "CommonConfig.h"
#include "Status.h"

#include <array>
#include <stdio.h>
#include <cstring>

#define PWM_MAX_TIMERS (3u)
#define PWM_MAX_CHAN_PER_TIMER (4u)
#define PWM_MAX_CHANNELS (10u) //(PWM_MAX_CHAN_PER_TIMER*PWM_MAX_TIMERS)

#define PWM_BITS (16u)
#define PWM_NWORDS ((PWM_BITS + 1u) * PWM_MAX_CHAN_PER_TIMER)
#define PWM_DMA_BUFFER_LEN ((PWM_NWORDS+31u) & ~31u) // round up to nearest 32


//#define PWM_DMA_BUFFER_LEN 96 // multiple of 32 and must >= ((PWM_BITS + 1) * PWM_MAX_CHAN_PER_TIMER) = 68



#define PWM_CHAN_IGNORE (0xFFFFFFFF)

#define PWM_SERVO_MIN (1000)
#define PWM_SERVO_MAX (2000)

#define DSHOT_ESC_MIN (48)
#define DSHOT_ESC_MAX (2047)
#define DSHOT_HI (200000000 / PWM_DSHOT_RATE_HZ * 3 / 4)
#define DSHOT_LO (200000000 / PWM_DSHOT_RATE_HZ * 3 / 8)

#define PWM_DSHOT_RATE_HZ (300000.0) // baud rate
#define PWM_MKS_RATE_HZ (333.0)
#define PWM_STD_RATE_HZ (50.0)

typedef enum
{
  PWM_STANDARD,
  PWM_DSHOT,
  PWM_NONE
} pwm_type;



#define TIM_CHANNEL_NONE (0xFFFFFFFF)

typedef struct // __attribute__((__packed__))
{
  TIM_HandleTypeDef * htim;
  uint32_t chan;
} PwmChannel;

typedef struct // __attribute__((__packed__))
{
  TIM_HandleTypeDef * htim;
  pwm_type type;
  float rate;
} PwmTimer;


class Pwm : public Status
{
private:

  uint32_t timer_map_[PWM_MAX_TIMERS][PWM_MAX_CHAN_PER_TIMER];
  uint32_t timer_lookup_[PWM_MAX_CHANNELS];

  uint32_t timers_;
  uint32_t channels_;

public:
  std::array<PwmChannel,PWM_MAX_CHANNELS> channel_;
  std::array<PwmTimer,PWM_MAX_TIMERS> timer_;

  // we have to put this init() code here so the compiler can do the template thing
  template <size_t timers , size_t channels>
  uint32_t init(
      std::array< PwmTimer, timers> timer,
      std::array< PwmChannel, channels> channel
  )
  {
    snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Pwm");
    initializationStatus_ = DRIVER_OK;

    std::copy(timer.begin(), timer.end(), timer_.begin());
    std::copy(channel.begin(), channel.end(),channel_.begin());

    timers_ = timers<PWM_MAX_TIMERS?timers:PWM_MAX_TIMERS;
    channels_ = channels<PWM_MAX_CHANNELS?channels:PWM_MAX_CHANNELS;


    for (uint32_t i = 0; i<timers_; i++)
    {
      if (timer_[i].htim == nullptr) continue;
      TIM_HandleTypeDef * htim = timer_[i].htim;


      TIM_MasterConfigTypeDef sMasterConfig = {0};

      if (htim == &htim1) htim->Instance = TIM1;
      else if (htim == &htim3) htim->Instance = TIM3;
      else if (htim == &htim4) htim->Instance = TIM4;
      else if (htim == &htim8) htim->Instance = TIM8;
      else {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return DRIVER_HAL_ERROR;
      }

      float rate = timer_[i].rate;
      pwm_type type = timer_[i].type;

      if ((rate >= 150000) && (type == PWM_DSHOT)) // DSHOT
      {
        htim->Init.Prescaler = 0;
        htim->Init.Period = (uint64_t) 200000000 / rate - 1;
      } else if (( type == PWM_STANDARD) && ( rate < 490)) {
        htim->Init.Prescaler = 199;
        htim->Init.Period = (uint64_t) 1000000 / rate - 1;
      } else {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return DRIVER_HAL_ERROR;
      }

      htim->Init.CounterMode = TIM_COUNTERMODE_UP;
      htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      htim->Init.RepetitionCounter = 0;
      htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
      if (HAL_TIM_PWM_Init(htim) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return DRIVER_HAL_ERROR;
      }
      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
      if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return DRIVER_HAL_ERROR;
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
        if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK) {
          initializationStatus_ |= DRIVER_HAL_ERROR;
          return DRIVER_HAL_ERROR;
        }
      }
      HAL_TIM_MspPostInit(htim);
    }

    // configure channels
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // nominally no output
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    for (uint32_t i = 0; i < channels_ ; i++ )
    {
      if ( (channel_[i].htim == nullptr) || (channel_[i].chan == TIM_CHANNEL_NONE)) continue;

      if (HAL_TIM_PWM_ConfigChannel(channel_[i].htim, &sConfigOC,channel_[i].chan) != HAL_OK) {
        initializationStatus_ |= DRIVER_HAL_ERROR;
        return DRIVER_HAL_ERROR;
      }
    }

    // Make a timer channel type cross-references.


    for ( uint32_t ch = 0; ch <channels_; ch++ ) {
      timer_lookup_[ch] = 0xFFFFFFFF;
      if(channel_[ch].htim != nullptr) {
        for ( uint32_t i=0; i<timers_; i++ ) {
          if(channel_[ch].htim == timer_[i].htim) {
            timer_lookup_[ch] = i;
            break;
          }
        }
      }
    }

    for (uint32_t i = 0; i < timers_; i++) {
      for(uint32_t c=0; c<PWM_MAX_CHAN_PER_TIMER; c++ ) {
        timer_map_[i][c] = TIM_CHANNEL_NONE;
      }
    }

    for (uint32_t ch = 0; ch < channels_; ch++ )
    {
      if (channel_[ch].htim == nullptr) continue;
      for (uint32_t i = 0; i < timers; i++)
      {
        if (channel_[ch].htim == timer_[i].htim)
        {
          for (uint32_t c = 0; c < PWM_MAX_CHAN_PER_TIMER; c++)
          {
            if ( (channel_[ch].chan>>2) == c)
            {
              timer_map_[i][c] = ch;
              break;
            }
          }
        }
      }
    }







    return initializationStatus_;
  }

  //uint32_t channels(void) { return nChan_;}

  void updateConfig(const float * rate, uint32_t channels);

  void enable(uint32_t chan);
  void disable(uint32_t chan);
  void writeUs(uint32_t chan, uint32_t us);
  void write(uint32_t chan, float val);

  void write(float * output, uint32_t channels);

  // Note this sets the rate for all members of the block.
  void setRate(uint32_t chan, float rate);

};

#endif /* PWM_H_ */
