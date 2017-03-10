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

#include "mux.h"
#include "param.h"
#include "board.h"
#include "arming_fsm.h"
#include "common.h"

namespace rosflight
{

class Arming_FSM;
class Params;
class Mux;

class RC
{

public:
  typedef enum
  {
    PARALLEL_PWM,
    CPPM,
  } rc_type_t;


  typedef enum
  {
    RC_X,
    RC_Y,
    RC_Z,
    RC_F
  } rc_channel_enum_t;


  bool _calibrate_rc;
  void init_rc(Arming_FSM *_fsm, Board *_board, Params *_params, Mux *_mux);
  bool rc_switch(int16_t channel);
  bool receive_rc();
  bool rc_low(int16_t channel);
  bool rc_high(int16_t channel);


private:
  typedef struct
  {
    int16_t channel;
    int16_t direction;
  } rc_switch_t;

  typedef struct
  {
    uint16_t channel_param;
    uint16_t max_angle_param;
    uint16_t max_rate_param;
    uint16_t center_param;
    uint16_t bottom_param;
    uint16_t range_param;
    control_channel_t *control_channel_ptr;
  } rc_channel_t;

  Arming_FSM *fsm;
  Board *board;
  Params *params;
  Mux *mux;

  uint32_t time_of_last_stick_deviation = 0;
  uint32_t last_rc_receive_time = 0;

  rc_channel_t channels[4];
  rc_switch_t switches[4];

  void calibrate_rc();
  void init_switches();
  void interpret_command_values();
  void interpret_command_type();
  bool sticks_deviated(uint32_t now_ms);
};
}
#endif
