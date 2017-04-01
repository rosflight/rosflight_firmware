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
#include <stdbool.h>
#include <math.h>

#include <turbovec.h>
#include <turbotrig.h>

#include "param.h"
#include "sensors.h"

namespace rosflight
{

class Params;
class Sensors;

class Estimator
{

public:

  // Controller needs direct access to these values
  vector_t omega = {0.0, 0.0, 0.0};
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;

  void reset_state();
  void reset_adaptive_bias();
  void init_estimator(Params *_params, Sensors *_sensors);
  void run_estimator();

  inline float get_roll() {return roll;}
  inline float get_pitch() {return pitch;}
  inline float get_yaw() {return yaw;}
  inline vector_t get_angular_velocity() {return omega;}
  inline quaternion_t get_attitude() {return q;}
  inline uint64_t get_estimator_timestamp() {return now_us;}

private:
  Params *params_;
  Sensors *sensors_;

  uint64_t now_us = 0;

  quaternion_t q = {1.0, 0.0, 0.0, 0.0};


  vector_t w1 = {0.0, 0.0, 0.0};
  vector_t w2 = {0.0, 0.0, 0.0};
  vector_t wbar = {0.0, 0.0, 0.0};
  vector_t wfinal = {0.0, 0.0, 0.0};
  vector_t w_acc = {0.0, 0.0, 0.0};
  const vector_t g = {0.0f, 0.0f, -1.0f};
  vector_t b = {0.0, 0.0, 0.0};
  quaternion_t q_tilde = {1.0, 0.0, 0.0, 0.0};
  quaternion_t q_hat = {1.0, 0.0, 0.0, 0.0};
  uint64_t last_time = 0;

  bool mat_exp = false;
  bool quad_int = false;
  bool use_acc = true;

  vector_t _accel_LPF = {0.0, 0.0, 0.0};
  vector_t _gyro_LPF = {0.0, 0.0, 0.0};

  void run_LPF();
};

}
