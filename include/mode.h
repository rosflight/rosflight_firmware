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

#pragma once

#include <stdint.h>

#include "board.h"
#include "sensors.h"
#include "param.h"
#include "rc.h"

namespace rosflight
{

class Mode
{

private:

  typedef enum
  {
    DISARMED = 0,
    ARMED = 1
  } armed_state_t;

  typedef enum
  {
    NORMAL = 0,
    FAILSAFE = 1
  } failsafe_state_t;

  RC *rc_;
  Board *board_;
  Params *params_;

  // We need a sensors pointer for gyro calibration on arm
  Sensors *sensors_;

  uint32_t prev_time_ms;
  uint32_t time_sticks_have_been_in_arming_position_ms = 0;

  armed_state_t _armed_state;
  failsafe_state_t _failsafe_state;

  bool started_gyro_calibration;

  bool arm(void);
  void disarm(void);
  bool check_failsafe(void);


public:
  Mode();

  void init_mode(Board *_board, Sensors *_sensors, Params *_params, RC *_rc);
  bool update_armed_state();

  inline bool armed() {return _armed_state;}
  inline bool in_failsafe() {return _failsafe_state;}

};

}
