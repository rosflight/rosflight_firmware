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

#include "Pwm.h"
#include "BoardConfig.h"

#include <stdio.h>
#include <cstring>


// Notes on DSHOT
// DSHOT  Bitrate	T1H      T0H    Bit(µs)	Frame (µs)
//  150	  150kbit/s	5.00	2.50	6.67	106.72
//  300	  300kbit/s	2.50	1.25	3.33	53.28 < we use this
//  600	  600kbit/s	1.25	0.625	1.67	26.72
// 1200	 1200kbit/s	0.625	0.313	0.83	13.28

//DATA_RAM PwmBlockStructure pwm_init[MAX_PWM_TIMER_BLOCKS] = PWM_INIT_DEFINE;

DMA_RAM uint32_t pwm_dma_buf[PWM_MAX_TIMERS][PWM_DMA_BUFFER_LEN];

uint32_t Pwm::init(void)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Pwm");
  initializationStatus_ = DRIVER_OK;

//  dmaBuf_ = pwm_dma_buf;

  for (uint32_t i = 0; i<PWM_MAX_TIMERS; i++)
  {
    if (timer_[i].htim == nullptr) continue;
    TIM_HandleTypeDef * htim = htim_[i];


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

  for (uint32_t i = 0; i < PWM_MAX_CHANNELS ; i++ )
  {
    if ( (channel_[i].htim == nullptr) || (channel_[i].chan == TIM_CHANNEL_NONE)) continue;

    if (HAL_TIM_PWM_ConfigChannel(channel_[i].htim, &sConfigOC,channel_[i].chan) != HAL_OK) {
      initializationStatus_ |= DRIVER_HAL_ERROR;
      return DRIVER_HAL_ERROR;
    }
  }

  // Make a timer channel type cross-references.


  for ( uint32_t ch = 0; ch <PWM_MAX_CHANNELS; ch++ ) {
    timer_lookup_[ch] = 0xFFFFFFFF;
    if(channel_[ch].htim != nullptr) {
      for ( uint32_t i=0; i<PWM_MAX_CHAN_PER_TIMER; i++ ) {
        if(channel_[ch].htim == timer_[i].htim) {
          timer_lookup_[ch] = i;
          break;
        }
      }
    }
  }

  for (uint32_t i = 0; i < PWM_MAX_TIMERS; i++) {
    for(uint32_t ch=0; ch<PWM_MAX_CHAN_PER_TIMER; ch++ ) {
      timer_map_[i][ch] = TIM_CHANNEL_NONE;
    }
  }

  for (uint32_t ch = 0; ch < PWM_MAX_CHANNELS; ch++ ) {
    if (channel_[ch].htim == nullptr) continue;

    for (uint32_t i = 0; i < PWM_MAX_TIMERS; i++) {
      if (channel_[ch].htim == timer_[i].htim) {
        timer_map_[i][ch] = channel_[ch].chan;
      }
    }
  }

  return initializationStatus_;
}

void Pwm::updateConfig(const float * rate, uint32_t channels) {
  channels = (channels < PWM_MAX_CHANNELS) ? channels : PWM_MAX_CHANNELS;

  for (uint32_t ch = 0; ch < channels; ch++) HAL_TIM_PWM_Stop(htim_[ch], chan_[ch]);

  for (uint32_t ch = 0; ch < channels; ch++) setRate(ch, rate[ch]);

  for (uint32_t ch = 0; ch < channels; ch++) HAL_TIM_PWM_Start(htim_[ch], chan_[ch]);
}

void Pwm::enable(uint32_t chan)
{
  if ( (chan < PWM_MAX_CHANNELS)
       && (channel_[chan].htim != nullptr)
       && (channel_[chan].chan != TIM_CHANNEL_NONE)
  ){
    HAL_TIM_PWM_Start(channel_[chan].htim, channel_[chan].chan);
  }
}
void Pwm::disable(uint32_t chan)
{
  if ( (chan < PWM_MAX_CHANNELS)
       && (channel_[chan].htim != nullptr)
       && (channel_[chan].chan != TIM_CHANNEL_NONE)
  ){
    HAL_TIM_PWM_Stop(channel_[chan].htim, channel_[chan].chan);
  }
}
void Pwm::writeUs(uint32_t chan, uint32_t us)
{
  if ( (chan < PWM_MAX_CHANNELS)
       && (channel_[chan].htim != nullptr)
       && (channel_[chan].chan != TIM_CHANNEL_NONE)
  ){
    us = (us < PWM_SERVO_MIN) ? PWM_SERVO_MIN : us;
    us = (us > PWM_SERVO_MAX) ? PWM_SERVO_MAX : us;
    __HAL_TIM_SET_COMPARE(channel_[chan].htim, channel_[chan].chan, us);
  }
}
void Pwm::write(uint32_t chan, float val)
{
  if (chan < PWM_MAX_CHANNELS) {
    val = (val < 0) ? 0 : val;
    val = (val > 1) ? 1 : val;
    uint32_t us = val * (float) (PWM_SERVO_MAX - PWM_SERVO_MIN) + PWM_SERVO_MIN;
    writeUs(chan, us);
  }
}

void Pwm::write(float * output, uint32_t channels)
{
  channels = (channels < PWM_MAX_CHANNELS) ? channels : PWM_MAX_CHANNELS;


  // Do this one timer at a time
  for (uint32_t i=0; i<PWM_MAX_TIMERS; i++){
    TIM_HandleTypeDef * htim = timer_[i].htim;
    if( htim == nullptr) continue;

    if ( timer_[i].type == PWM_STANDARD) {
      for (uint32_t c=0; c < PWM_MAX_CHAN_PER_TIMER; c++ ){
        uint32_t ch = timer_map_[i][c];  // TIM_CHANNEL_[1-6 or NONE]
        if (ch < channels) write(ch,output[c]);
      }
    } else if ( timer_[i].type == PWM_DSHOT) {
     HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);
     for (uint32_t c=0; c < PWM_MAX_CHAN_PER_TIMER; c++ ){
        uint32_t ch = timer_map_[i][c];  // TIM_CHANNEL_[1-6 or NONE]
        if (ch < channels) {
          float val = output[c];
          val = (val < 0) ? 0 : val;
          val = (val > 1) ? 1 : val;
          uint32_t value = val * (float) (DSHOT_ESC_MAX - DSHOT_ESC_MIN) + DSHOT_ESC_MIN;
          value = (uint32_t) (value) << 1;
          uint32_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x000F;
          value = (value << 4) | crc;
          uint32_t mask = 0x00008000;
          uint32_t * buf = pwm_dma_buf[i] + c; // ch is differeht here.
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
          *(buf += 4) = (value & mask) ? DSHOT_HI : DSHOT_LO;
          *buf = 0;
        }
      }
      #define NWORDS ((16 + 1) * 4) // 4 channels 16 data bits + 1 end of message bit
      HAL_TIM_DMABurst_MultiWriteStart(
          htim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
          (uint32_t *) (pwm_dma_buf[i]),TIM_DMABURSTLENGTH_4TRANSFERS, NWORDS
      );
    }
  }

//
//  for (uint32_t bk = 0; bk < nBlocks_; bk++) {
//    TIM_HandleTypeDef * htim = block_[bk].htim;
//    if (block_[bk].type == PWM_STANDARD) {
////      for (uint32_t ch = 0; ch < 4; ch++) {
////        uint32_t output_index = block_[bk].chan[ch];
////        if (output_index < nChan_) {
////          float val = output[output_index];
////
////          val = (val < 0) ? 0 : val;
////          val = (val > 1) ? 1 : val;
////          uint32_t us = val * (float) (PWM_SERVO_MAX - PWM_SERVO_MIN) + PWM_SERVO_MIN;
////          __HAL_TIM_SET_COMPARE(htim, ch << 2, us);
////        } else {
////          __HAL_TIM_SET_COMPARE(htim, ch << 2, 0);
////        }
////      }
//
//
//    } else { // DSHOT
//      #define NWORDS ((16 + 1) * 4) // 4 channels 16 data bits + 1 end of message bit
//      HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);
//
//      // memset(dmaBuf_[bk],0, NWORDS*sizeof(uint32_t));
//      for (uint32_t ch = 0; ch < 4; ch++) {
//        uint32_t output_index = block_[bk].chan[ch];
//        if (output_index < nChan_) {
//          float val = output[output_index];
//          val = (val < 0) ? 0 : val;
//          val = (val > 1) ? 1 : val;
//          uint32_t value = val * (float) (DSHOT_ESC_MAX - DSHOT_ESC_MIN) + DSHOT_ESC_MIN;
//          value = (uint32_t) (value) << 1;
//          uint32_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x000F;
//          value = (value << 4) | crc;
//          uint32_t mask = 0x00008000;
//          uint32_t * buf = dmaBuf_[bk] + ch;
//
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & (mask >>= 1)) ? DSHOT_HI : DSHOT_LO;
//          *(buf += 4) = (value & mask) ? DSHOT_HI : DSHOT_LO;
//          *buf = 0;
//        }
//      }
//      HAL_TIM_DMABurst_MultiWriteStart(htim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, (uint32_t *) (dmaBuf_[bk]),
//                                       TIM_DMABURSTLENGTH_4TRANSFERS, NWORDS);
//    }
//  }
}

// Note this sets the rate for all members of the block.
void Pwm::setRate(uint32_t ch, float rate)
{
  if (ch > PWM_MAX_CHANNELS) return;

  uint32_t indx = timer_lookup_[ch];
  if ((rate >= 0.0) && (rate <= 490.0)) {
    timer_[indx].type = PWM_STANDARD;
    timer_[indx].rate = rate;
    timer_[indx].htim->Instance->PSC = 199;
    timer_[indx].htim->Instance->ARR = (uint32_t) (1000000.0 / rate + 0.99); // assumes dividers are set for 1MHz clock input
  } else if ((rate >= 150000.0) && (rate <= 1200000.0)) {
    timer_[indx].type = PWM_DSHOT;
    timer_[indx].rate = rate;
    timer_[indx].htim->Instance->PSC = 0;
    timer_[indx].htim->Instance->ARR = (uint32_t) (200000000.0 / rate + 0.99); // assumes dividers are set for 200MHz clock input
  }

}
