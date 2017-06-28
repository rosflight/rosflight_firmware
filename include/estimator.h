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

namespace rosflight_firmware
{

class ROSflight;

class Estimator
{

private:
  // Controller needs direct access to these values
  vector_t omega_ = {0.0, 0.0, 0.0};
  float roll_ = 0.0;
  float pitch_ = 0.0;
  float yaw_ = 0.0;
  quaternion_t q_ = {1.0, 0.0, 0.0, 0.0};

  ROSflight& RF_;

  uint64_t now_us_ = 0;
  uint64_t last_acc_update_us_ = 0;

  vector_t w1_ = {0.0, 0.0, 0.0};
  vector_t w2_ = {0.0, 0.0, 0.0};
  vector_t wbar_ = {0.0, 0.0, 0.0};
  vector_t wfinal_ = {0.0, 0.0, 0.0};
  vector_t w_acc_ = {0.0, 0.0, 0.0};
  const vector_t g_ = {0.0f, 0.0f, -1.0f};
  vector_t bias_ = {0.0, 0.0, 0.0};
  quaternion_t q_tilde_ = {1.0, 0.0, 0.0, 0.0};
  quaternion_t q_hat_ = {1.0, 0.0, 0.0, 0.0};
  uint64_t last_time_ = 0;

  vector_t accel_LPF_ = {0.0, 0.0, 0.0};
  vector_t gyro_LPF_ = {0.0, 0.0, 0.0};

  void run_LPF();

public:

  Estimator(ROSflight& _rf);

  void reset_state();
  void reset_adaptive_bias();
  void init();
  void run_estimator();

  inline float get_roll()
  {
    return roll_;
  }
  inline float get_pitch()
  {
    return pitch_;
  }
  inline float get_yaw()
  {
    return yaw_;
  }
  inline vector_t get_angular_velocity()
  {
    return omega_;
  }
  inline quaternion_t get_attitude()
  {
    return q_;
  }
  inline uint64_t get_estimator_timestamp()
  {
    return now_us_;
  }
};

}
