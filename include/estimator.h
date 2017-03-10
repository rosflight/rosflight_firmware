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
#include <math.h>

#include <turbovec.h>
#include <turbotrig.h>

#include "param.h"
#include "sensors.h"

namespace rosflight {

class Estimator
{

public:
  quaternion_t q;
  vector_t omega;
  float roll;
  float pitch;
  float yaw;
  float altitude;
  uint64_t now_us;
  vector_t _adaptive_gyro_bias;

  void reset_state();
  void reset_adaptive_bias();
  void init_estimator(Params *_params, Sensors *_sensors);
  void run_estimator();

private:
  Params* params_;
  Sensors* sensors_;

  vector_t w1;
  vector_t w2;
  vector_t wbar;
  vector_t wfinal;
  vector_t w_acc;
  const vector_t g = {0.0f, 0.0f, -1.0f};
  vector_t b;
  quaternion_t q_tilde;
  quaternion_t q_hat;
  uint64_t last_time;

  bool mat_exp;
  bool quad_int;
  bool use_acc;

  vector_t _accel_LPF;
  vector_t _gyro_LPF;

  void run_LPF();
};

}
