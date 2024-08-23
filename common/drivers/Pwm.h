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

#include <BoardConfig.h>

#include <Status.h>

#define PWM_DMA_BUFFER_LEN 96

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
  PWM_DSHOT
} pwm_type;

typedef struct __attribute__((__packed__))
{
  TIM_HandleTypeDef * htim;
  pwm_type type;
  float rate;
  uint32_t chan[4];
} PwmBlockStructure;

class Pwm : public Status
{
public:
  Pwm() { initializationStatus_ = DRIVER_NOT_INITIALIZED; }
  bool initGood(void) { return initializationStatus_ == DRIVER_OK; }

  uint32_t init(void);
  void updateConfig(const float * rate, uint32_t channels);

  void enable(uint32_t chan)
  {
    if (chan < PWM_CHANNELS) {
      TIM_HandleTypeDef * htim = htim_[chan];
      if (htim != nullptr) HAL_TIM_PWM_Start(htim, chan_[chan]);
    }
  }
  void disable(uint32_t chan)
  {
    if (chan < PWM_CHANNELS) {
      TIM_HandleTypeDef * htim = htim_[chan];
      if (htim != nullptr) HAL_TIM_PWM_Stop(htim, chan_[chan]);
    }
  }
  void writeUs(uint32_t chan, uint32_t us)
  {
    if (chan < PWM_CHANNELS) {
      TIM_HandleTypeDef * htim = htim_[chan];
      if (htim != nullptr) {
        us = (us < PWM_SERVO_MIN) ? PWM_SERVO_MIN : us;
        us = (us > PWM_SERVO_MAX) ? PWM_SERVO_MAX : us;
        __HAL_TIM_SET_COMPARE(htim, chan_[chan], us);
      }
    }
  }
  void write(uint32_t chan, float val)
  {
    if (chan < PWM_CHANNELS) {
      val = (val < 0) ? 0 : val;
      val = (val > 1) ? 1 : val;
      uint32_t us = val * (float) (PWM_SERVO_MAX - PWM_SERVO_MIN) + PWM_SERVO_MIN;
      writeUs(chan, us);
    }
  }

  void write(float * output, uint32_t channels)
  {
    channels = (channels < PWM_CHANNELS) ? channels : PWM_CHANNELS;

    for (uint32_t bk = 0; bk < PWM_TIMER_BLOCKS; bk++) {
      TIM_HandleTypeDef * htim = block_[bk].htim;

      if (block_[bk].type == PWM_STANDARD) {
        for (uint32_t ch = 0; ch < 4; ch++) {
          uint32_t output_index = block_[bk].chan[ch];
          if (output_index < PWM_CHANNELS) {
            float val = output[output_index];

            val = (val < 0) ? 0 : val;
            val = (val > 1) ? 1 : val;
            uint32_t us = val * (float) (PWM_SERVO_MAX - PWM_SERVO_MIN) + PWM_SERVO_MIN;
            __HAL_TIM_SET_COMPARE(htim, ch << 2, us);
          } else {
            __HAL_TIM_SET_COMPARE(htim, ch << 2, 0);
          }
        }
      } else // DSHOT
      {
#define NWORDS ((16 + 1) * 4) // 4 channels 16 data bits + 1 end of message bit
        HAL_TIM_DMABurst_WriteStop(htim, TIM_DMA_UPDATE);

        // memset(dmaBuf_[bk],0, NWORDS*sizeof(uint32_t));
        for (uint32_t ch = 0; ch < 4; ch++) {
          uint32_t output_index = block_[bk].chan[ch];
          if (output_index < PWM_CHANNELS) {
            float val = output[output_index];
            val = (val < 0) ? 0 : val;
            val = (val > 1) ? 1 : val;
            uint32_t value = val * (float) (DSHOT_ESC_MAX - DSHOT_ESC_MIN) + DSHOT_ESC_MIN;
            value = (uint32_t) (value) << 1;
            uint32_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x000F;
            value = (value << 4) | crc;
            uint32_t mask = 0x00008000;
            uint32_t * buf = dmaBuf_[bk] + ch;

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
        HAL_TIM_DMABurst_MultiWriteStart(htim, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, (uint32_t *) (dmaBuf_[bk]),
                                         TIM_DMABURSTLENGTH_4TRANSFERS, NWORDS);
      }
    }
  }

  // Note this sets the rate for all members of the block.
  void setRate(uint32_t chan, float rate)
  {
    if (chan < PWM_CHANNELS) {
      uint32_t block_index = blockIndex_[chan];
      if (block_index < PWM_TIMER_BLOCKS) {
        PwmBlockStructure & block = block_[block_index];

        if ((rate >= 0.0) && (rate <= 490.0)) {
          block.type = PWM_STANDARD;
          block.rate = rate;
          block.htim->Instance->PSC = 199;
          block.htim->Instance->ARR =
            (uint32_t) (1000000.0 / rate + 0.99); // assumes dividers are set for 1MHz clock input
        } else if ((rate >= 150000.0) && (rate <= 1200000.0)) {
          block.type = PWM_DSHOT;
          block.rate = rate;
          block.htim->Instance->PSC = 0;
          block.htim->Instance->ARR =
            (uint32_t) (200000000.0 / rate + 0.99); // assumes dividers are set for 200MHz clock input
        }
      }
    }
  }

private:
  PwmBlockStructure * block_;
  TIM_HandleTypeDef * htim_[PWM_CHANNELS];
  uint32_t chan_[PWM_CHANNELS];
  uint32_t (*dmaBuf_)[PWM_DMA_BUFFER_LEN];
  uint32_t blockIndex_[PWM_CHANNELS];
  uint32_t initializationStatus_ = 0;
};

#endif /* PWM_H_ */
