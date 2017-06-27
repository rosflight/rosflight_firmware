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
#include <stdbool.h>

#include "mixer.h"

namespace rosflight_firmware
{

class ROSflight;

class PID
{
public:
  PID(ROSflight &_rf);
  void init(float kp, float ki, float kd, float max, float min, float tau);
  float run(float x, float x_c, float dt, bool use_derivative, float xdot =0 );

private:
  ROSflight& RF_;
  float kp_ = 0.0;
  float ki_ = 0.0;
  float kd_ = 0.0;

  float max_ = 1.0;
  float min_ = -1.0;

  float integrator_ = 0.0;
  float prev_x_ = 0.0;
  float differentiator_ = 0.0;
  float tau_ = 0.05;

};

class Controller
{

private:

  Mixer::command_t outputs_;

  PID roll_;
  PID roll_rate_;
  PID pitch_;
  PID pitch_rate_;
  PID yaw_rate_;

  ROSflight& RF_;

  float prev_time;

public:
  Controller(ROSflight& _rf);
  void run_controller();
  void init();
  void calculate_equilbrium_torque_from_rc();
  void param_change_callback(uint16_t param_id);
  inline Mixer::command_t get_outputs() {return outputs_;}

};

}
