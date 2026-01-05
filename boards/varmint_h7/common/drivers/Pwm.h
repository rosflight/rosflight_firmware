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

#include "BoardConfig.h"
#include "Status.h"

#include <array>


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
  PWM_DSHOT,
  PWM_NONE
} pwm_type;

#define PWM_MAX_TIMERS (3)
#define PWM_MAX_CHAN_PER_TIMER (4)
#define PWM_MAX_CHANNELS (10) //(PWM_MAX_CHAN_PER_TIMER*PWM_MAX_TIMERS)

#define TIM_CHANNEL_NONE (0xFFFFFFFF)

//typedef struct // __attribute__((__packed__))
//{
//  TIM_HandleTypeDef * htim;
//  pwm_type type;
//  float rate;
//  uint32_t chan[PWM_MAX_CHAN_PER_TIMER];
//} PwmBlockStructure;



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

  uint32_t blockIndex_[PWM_MAX_CHANNELS];
  TIM_HandleTypeDef * htim_[PWM_MAX_CHANNELS];
  uint32_t chan_[PWM_MAX_CHANNELS];

  uint32_t (*dmaBuf_)[PWM_DMA_BUFFER_LEN];


public:
  std::array<PwmChannel,PWM_MAX_CHANNELS> channel_;
  std::array<PwmTimer,PWM_MAX_TIMERS> timer_;
  uint32_t init(void);

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
