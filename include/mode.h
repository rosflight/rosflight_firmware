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

#include <stdint.h>

#include "board.h"
#include "sensors.h"
#include "param.h"
#include "rc.h"

namespace rosflight
{

class Sensors;
class Mode
{

public:

  enum
  {
    ERROR_NONE = 0x0000,
    ERROR_INVALID_MIXER = 0x0001,
    ERROR_IMU_NOT_RESPONDING = 0x0002,
    ERROR_RC_LOST = 0x0004,
    ERROR_UNHEALTHY_ESTIMATOR = 0x0008,
    ERROR_TIME_GOING_BACKWARDS = 0x0010,
    ERROR_UNCALIBRATED_IMU = 0x0020,
  };

private:

  RC *rc_;
  Board *board_;
  Params *params_;

  // We need a sensors pointer for gyro calibration on arm
  Sensors *sensors_;

  uint32_t prev_time_ms;
  uint32_t time_sticks_have_been_in_arming_position_ms = 0;

  bool _armed;
  bool _failsafe_active;
  uint16_t _error_code;

  bool started_gyro_calibration;

  bool arm(void);
  void disarm(void);
  bool check_failsafe(void);

public:

  Mode();

  void init_mode(Board *_board, Sensors *_sensors, Params *_params, RC *_rc);
  bool update_state();

  bool armed() { return _armed; }
  bool in_failsafe() { return _failsafe_active; }
  uint16_t error_state(){ return _error_code; }
  void set_error_code(uint16_t error_code) { _error_code |= error_code; }
  void clear_error_code(uint16_t error_code) { _error_code &= ~(error_code); }
};

}
