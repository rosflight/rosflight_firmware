/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab, Provo UT
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 */

#ifndef RC_H_
#define RC_H_

#include <stdint.h>
#include <stdbool.h>

#include "param.h"
#include "board.h"
#include "common.h"

namespace rosflight
{

class Params;

typedef enum
{
  RC_STICK_X,
  RC_STICK_Y,
  RC_STICK_Z,
  RC_STICK_F,
  RC_STICKS_COUNT
} rc_stick_t;

typedef enum
{
  RC_SWITCH_ARM,
  RC_SWITCH_ATT_OVERRIDE,
  RC_SWITCH_THROTTLE_OVERRIDE,
  RC_SWITCH_ATT_TYPE,
  RC_SWITCHES_COUNT
} rc_switch_t;

class RC
{

public:

  typedef enum
  {
    PARALLEL_PWM,
    CPPM,
  } rc_type_t;

  void init(Board *_board, Params *_params);
  void init_rc();
  float rc_stick(rc_stick_t channel);
  bool rc_switch(int16_t channel);
  bool rc_switch_mapped(rc_switch_t channel);
  bool receive_rc();
  bool new_command();
  void param_change_callback(uint16_t param_id);

private:
  typedef struct
  {
    uint8_t channel;
    int8_t direction;
    bool mapped;
  } rc_switch_config_t;

  typedef struct
  {
    uint8_t channel;
    bool one_sided;
  } rc_stick_config_t;

  Board *board;
  Params *params;
  bool new_command_;

  uint32_t time_of_last_stick_deviation = 0;
  uint32_t last_rc_receive_time = 0;

  rc_stick_config_t sticks[RC_STICKS_COUNT];
  rc_switch_config_t switches[RC_SWITCHES_COUNT];

  bool switch_values[RC_SWITCHES_COUNT];
  float stick_values[RC_STICKS_COUNT];

  void init_switches();
  void init_sticks();
};
}
#endif
