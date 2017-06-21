/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mode.h"
#include "param.h"
#include "board.h"
#include "common.h"
#include "rc.h"

namespace rosflight
{

class Mode;

class Mux
{
private:
  typedef enum
  {
    ATT_MODE_RATE,
    ATT_MODE_ANGLE
  } att_mode_t;

  enum
  {
    MUX_X,
    MUX_Y,
    MUX_Z,
    MUX_F,
  };

  typedef struct
  {
    rc_stick_t rc_channel;
    uint32_t last_override_time;
  } rc_stick_override_t;

  rc_stick_override_t rc_stick_override[3] =
  {
    { RC_STICK_X, 0 },
    { RC_STICK_Y, 0 },
    { RC_STICK_Z, 0 }
  };

  typedef struct
  {
    control_channel_t *rc;
    control_channel_t *onboard;
    control_channel_t *combined;
  } mux_t;

public:

  mux_t muxes[4] =
  {
    {&_rc_control.x, &_offboard_control.x, &_combined_control.x},
    {&_rc_control.y, &_offboard_control.y, &_combined_control.y},
    {&_rc_control.z, &_offboard_control.z, &_combined_control.z},
    {&_rc_control.F, &_offboard_control.F, &_combined_control.F}
  };

  control_t _rc_control =
  {
    0,
    {false, ANGLE, 0.0},
    {false, ANGLE, 0.0},
    {false, RATE, 0.0},
    {false, THROTTLE, 0.0}
  };
  control_t _offboard_control =
  {
    0,
    {false, ANGLE, 0.0},
    {false, ANGLE, 0.0},
    {false, RATE, 0.0},
    {false, THROTTLE, 0.0}
  };
  control_t _combined_control =
  {
    0,
    {false, ANGLE, 0.0},
    {false, ANGLE, 0.0},
    {false, RATE, 0.0},
    {false, THROTTLE, 0.0}
  };
  control_t _failsafe_control =
  {
    0,
    {true, ANGLE, 0.0},
    {true, ANGLE, 0.0},
    {true, RATE, 0.0},
    {true, THROTTLE, 0.0}
  };

  bool mux_inputs();
  bool rc_override_active();
  bool offboard_control_active();
  void signal_new_command();
  void init(Mode *_fsm, Params *_params, Board *_board, RC *_rc, CommLink *_commlink);

private:

  Board *board;
  Mode *fsm;
  Params *params;
  RC *rc;
  CommLink *commlink;

  bool new_command;
  bool rc_override;

  void do_muxing(uint8_t mux_channel);
  void do_min_throttle_muxing();
  void interpret_rc(void);
  bool stick_deviated(uint8_t channel);
  bool do_roll_pitch_yaw_muxing(uint8_t channel);
  bool do_throttle_muxing(void);
};

}
