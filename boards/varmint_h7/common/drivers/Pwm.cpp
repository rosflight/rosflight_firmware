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
#include "CommonConfig.h"

#include <stdio.h>
#include <cstring>


// Notes on DSHOT
// DSHOT  Bitrate	T1H      T0H    Bit(µs)	Frame (µs)
//  150	  150kbit/s	5.00	2.50	6.67	106.72
//  300	  300kbit/s	2.50	1.25	3.33	53.28 < we use this
//  600	  600kbit/s	1.25	0.625	1.67	26.72
// 1200	 1200kbit/s	0.625	0.313	0.83	13.28

DMA_RAM uint32_t pwm_dma_buf[PWM_MAX_TIMERS][PWM_DMA_BUFFER_LEN];

void Pwm::updateConfig(const float * rate, uint32_t channels) {
  channels = (channels < channels_) ? channels : channels_;

  for (uint32_t ch = 0; ch < channels; ch++) HAL_TIM_PWM_Stop(channel_[ch].htim, channel_[ch].chan);

  for (uint32_t ch = 0; ch < channels; ch++) setRate(ch, rate[ch]);

  for (uint32_t ch = 0; ch < channels; ch++) HAL_TIM_PWM_Start(channel_[ch].htim, channel_[ch].chan);
}

void Pwm::enable(uint32_t chan)
{
  if ( (chan < channels_)
       && (channel_[chan].htim != nullptr)
       && (channel_[chan].chan != TIM_CHANNEL_NONE)
  ){
    HAL_TIM_PWM_Start(channel_[chan].htim, channel_[chan].chan);
  }
}
void Pwm::disable(uint32_t chan)
{
  if ( (chan < channels_)
       && (channel_[chan].htim != nullptr)
       && (channel_[chan].chan != TIM_CHANNEL_NONE)
  ){
    HAL_TIM_PWM_Stop(channel_[chan].htim, channel_[chan].chan);
  }
}
void Pwm::writeUs(uint32_t chan, uint32_t us)
{
  if ( (chan < channels_)
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
  if (chan < channels_) {
    val = (val < 0) ? 0 : val;
    val = (val > 1) ? 1 : val;
    uint32_t us = val * (float) (PWM_SERVO_MAX - PWM_SERVO_MIN) + PWM_SERVO_MIN;
    writeUs(chan, us);
  }
}

void Pwm::write(float * output, uint32_t channels)
{
  channels = (channels < channels_) ? channels : channels_;


  // Do this one timer at a time
  for (uint32_t i=0; i<timers_; i++){
    TIM_HandleTypeDef * htim = timer_[i].htim;
    if( htim == nullptr) continue;

    if ( timer_[i].type == PWM_STANDARD) {
      for (uint32_t c=0; c < PWM_MAX_CHAN_PER_TIMER; c++ ){
        uint32_t ch = timer_map_[i][c];  // TIM_CHANNEL_[1-6 or NONE]
        if (ch < channels) write(ch,output[ch]);
      }
    } else if ( timer_[i].type == PWM_DSHOT) {
     HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);
     for (uint32_t c=0; c < PWM_MAX_CHAN_PER_TIMER; c++ ){
        uint32_t ch = timer_map_[i][c];  // TIM_CHANNEL_[1-6 or NONE]
        if (ch < channels) {
          float val = output[ch];
          val = (val < 0) ? 0 : val;
          val = (val > 1) ? 1 : val;
          uint32_t value = val * (float) (DSHOT_ESC_MAX - DSHOT_ESC_MIN) + DSHOT_ESC_MIN;
          value = (uint32_t) (value) << 1;
          uint32_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x000F;
          value = (value << 4) | crc;
          uint32_t mask = 0x00008000;
          uint32_t * buf = pwm_dma_buf[i] + c;
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
      //#define PWM_NWORDS ((16 + 1) * 4) // 4 channels 16 data bits + 1 end of message bit
                                    // ((PWM_BITS + 1) * PWM_MAX_CHAN_PER_TIMER
      HAL_TIM_DMABurst_MultiWriteStart(
          htim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
          (uint32_t *) (pwm_dma_buf[i]),TIM_DMABURSTLENGTH_4TRANSFERS, PWM_NWORDS
      );
    }
  }


//  for (uint32_t bk = 0; bk < nBlocks_; bk++) {
//    TIM_HandleTypeDef * htim = block_[bk].htim;
//    if (block_[bk].type == PWM_STANDARD) {
//      for (uint32_t ch = 0; ch < 4; ch++) {
//        uint32_t output_index = block_[bk].chan[ch];
//        if (output_index < nChan_) {
//          float val = output[output_index];
//
//          val = (val < 0) ? 0 : val;
//          val = (val > 1) ? 1 : val;
//          uint32_t us = val * (float) (PWM_SERVO_MAX - PWM_SERVO_MIN) + PWM_SERVO_MIN;
//          __HAL_TIM_SET_COMPARE(htim, ch << 2, us);
//        } else {
//          __HAL_TIM_SET_COMPARE(htim, ch << 2, 0);
//        }
//      }
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
  if (ch > channels_) return;

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
