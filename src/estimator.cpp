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

#include "estimator.h"
#include "rosflight.h"

namespace rosflight_firmware
{

Estimator::Estimator(ROSflight &_rf):
  RF_(_rf)
{}

void Estimator::reset_state()
{
  state_.attitude.w = 1.0f;
  state_.attitude.x = 0.0f;
  state_.attitude.y = 0.0f;
  state_.attitude.z = 0.0f;

  state_.angular_velocity.x = 0.0f;
  state_.angular_velocity.y = 0.0f;
  state_.angular_velocity.z = 0.0f;

  state_.roll = 0.0f;
  state_.pitch = 0.0f;
  state_.yaw = 0.0f;

  w1_.x = 0.0f;
  w1_.y = 0.0f;
  w1_.z = 0.0f;

  w2_.x = 0.0f;
  w2_.y = 0.0f;
  w2_.z = 0.0f;

  bias_.x = 0.0f;
  bias_.y = 0.0f;
  bias_.z = 0.0f;

  accel_LPF_.x = 0;
  accel_LPF_.y = 0;
  accel_LPF_.z = -9.80665;

  gyro_LPF_.x = 0;
  gyro_LPF_.y = 0;
  gyro_LPF_.z = 0;

  state_.timestamp_us = RF_.board_.clock_micros();

  extatt_update_next_run_ = false;

  // Clear the unhealthy estimator flag
  RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
}

void Estimator::reset_adaptive_bias()
{
  bias_.x = 0;
  bias_.y = 0;
  bias_.z = 0;
}

void Estimator::init()
{
  last_time_ = 0;
  last_acc_update_us_ = 0;
  last_extatt_update_us_ = 0;
  reset_state();
}

void Estimator::param_change_callback(uint16_t param_id)
{
  (void) param_id;
}

void Estimator::run_LPF()
{
  float alpha_acc = RF_.params_.get_param_float(PARAM_ACC_ALPHA);
  const turbomath::Vector &raw_accel = RF_.sensors_.data().accel;
  accel_LPF_.x = (1.0f-alpha_acc)*raw_accel.x + alpha_acc*accel_LPF_.x;
  accel_LPF_.y = (1.0f-alpha_acc)*raw_accel.y + alpha_acc*accel_LPF_.y;
  accel_LPF_.z = (1.0f-alpha_acc)*raw_accel.z + alpha_acc*accel_LPF_.z;

  float alpha_gyro_xy = RF_.params_.get_param_float(PARAM_GYRO_XY_ALPHA);
  float alpha_gyro_z = RF_.params_.get_param_float(PARAM_GYRO_Z_ALPHA);
  const turbomath::Vector &raw_gyro = RF_.sensors_.data().gyro;
  gyro_LPF_.x = (1.0f-alpha_gyro_xy)*raw_gyro.x + alpha_gyro_xy*gyro_LPF_.x;
  gyro_LPF_.y = (1.0f-alpha_gyro_xy)*raw_gyro.y + alpha_gyro_xy*gyro_LPF_.y;
  gyro_LPF_.z = (1.0f-alpha_gyro_z)*raw_gyro.z + alpha_gyro_z*gyro_LPF_.z;
}

void Estimator::set_external_attitude_update(const turbomath::Quaternion &q)
{
  extatt_update_next_run_ = true;
  q_extatt_ = q;
}

void Estimator::run()
{
  float kp, ki;
  uint64_t now_us = RF_.sensors_.data().imu_time;
  if (last_time_ == 0)
  {
    last_time_ = now_us;
    last_acc_update_us_ = now_us;
    last_extatt_update_us_ = now_us;
    return;
  }
  else if (now_us < last_time_)
  {
    // this shouldn't happen
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    last_time_ = now_us;
    return;
  }


  RF_.state_manager_.clear_error(StateManager::ERROR_TIME_GOING_BACKWARDS);

  float dt = (now_us - last_time_) * 1e-6f;
  last_time_ = now_us;
  state_.timestamp_us = now_us;

  // Crank up the gains for the first few seconds for quick convergence
  if (now_us < static_cast<uint64_t>(RF_.params_.get_param_int(PARAM_INIT_TIME))*1000)
  {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP_ACC)*10.0f;
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI)*10.0f;
  }
  else
  {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP_ACC);
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI);
  }

  // Run LPF to reject a lot of noise
  run_LPF();

  // add in accelerometer
  float a_sqrd_norm = accel_LPF_.sqrd_norm();

  // establish allowed acceleration deviation from 1g (i.e., non-accelerated flight)
  const float margin = RF_.params_.get_param_float(PARAM_FILTER_ACCEL_MARGIN);
  const float lowerbound = (1.0f - margin)*(1.0f - margin)*9.80665f*9.80665f;
  const float upperbound = (1.0f + margin)*(1.0f + margin)*9.80665f*9.80665f;

  turbomath::Vector w_err;
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC)
      && lowerbound < a_sqrd_norm && a_sqrd_norm < upperbound)
  {
    // Get error estimated by accelerometer measurement
    last_acc_update_us_ = now_us;
    // turn measurement into a unit vector
    turbomath::Vector a = accel_LPF_.normalized();
    // Get the quaternion from accelerometer (low-frequency measure q)
    // (Not in either paper)
    turbomath::Quaternion q_acc_inv(g_, a);
    // Get the error quaternion between observer and low-freq q
    // Below Eq. 45 Mahony Paper
    turbomath::Quaternion q_tilde = q_acc_inv * state_.attitude;
    // Correction Term of Eq. 47a and 47b Mahony Paper
    // w_err = 2*s_tilde*v_tilde
    w_err.x = -2.0f*q_tilde.w*q_tilde.x;
    w_err.y = -2.0f*q_tilde.w*q_tilde.y;
    w_err.z = 0.0f; // Don't correct z, because it's unobservable from the accelerometer
  }

  if (extatt_update_next_run_)
  {
    turbomath::Vector xhat_BW, yhat_BW, zhat_BW;
    turbomath::Vector xext_BW, yext_BW, zext_BW;
    {
      // create DCM from quaternion attitude estimate
      float &w = state_.attitude.w, &x = state_.attitude.x, &y = state_.attitude.y, &z = state_.attitude.z;
      xhat_BW.x = 1.0f - 2.0f*(y*y + z*z); xhat_BW.y = 2.0f*(x*y - z*w); xhat_BW.z = 2.0f*(x*z + y*w);
      yhat_BW.x = 2.0f*(x*y + z*w); yhat_BW.y = 1.0f - 2.0f*(x*x + z*z); yhat_BW.z = 2.0f*(y*z - x*w);
      zhat_BW.x = 2.0f*(x*z - y*w); zhat_BW.y = 2.0f*(y*z + x*w); zhat_BW.z = 1.0f - 2.0f*(x*x + y*y);
    }
    {
      // create DCM from quaternion external attitude
      float &w = q_extatt_.w, &x = q_extatt_.x, &y = q_extatt_.y, &z = q_extatt_.z;
      xext_BW.x = 1.0f - 2.0f*(y*y + z*z); xext_BW.y = 2.0f*(x*y - z*w); xext_BW.z = 2.0f*(x*z + y*w);
      yext_BW.x = 2.0f*(x*y + z*w); yext_BW.y = 1.0f - 2.0f*(x*x + z*z); yext_BW.z = 2.0f*(y*z - x*w);
      zext_BW.x = 2.0f*(x*z - y*w); zext_BW.y = 2.0f*(y*z + x*w); zext_BW.z = 1.0f - 2.0f*(x*x + y*y);
    }
    turbomath::Vector w_ext = xext_BW.cross(xhat_BW) + yext_BW.cross(yhat_BW) + zext_BW.cross(zhat_BW);

    // the angular rate correction from external attitude updates occur at a
    // different rate than IMU updates, so it needs to be integrated with a
    // different dt. The following scales the correction term by the timestep
    // ratio so that it is integrated correctly.
    const float extAttDt = (now_us - last_extatt_update_us_) * 1e-6f;
    const float scaleDt = (dt > 0) ? (extAttDt / dt) : 0.0f;
    w_ext *= scaleDt;

    // use error calculated from external attitude update only
    w_err = w_ext;
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP_EXT);

    last_extatt_update_us_ = now_us;
    extatt_update_next_run_ = false;
  }

  // integrate biases driven by measured angular error
  // eq 47b Mahony Paper, using correction term w_err found above
  bias_ -= ki*w_err*dt;

  // Handle Gyro Measurements
  turbomath::Vector wbar;
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_QUAD_INT))
  {
    // Quadratic Interpolation (Eq. 14 Casey Paper)
    // this step adds 12 us on the STM32F10x chips
    wbar = (w2_/-12.0f) + w1_*(8.0f/12.0f) + gyro_LPF_ * (5.0f/12.0f);
    w2_ = w1_;
    w1_ = gyro_LPF_;
  }
  else
  {
    wbar = gyro_LPF_;
  }

  // Build the composite omega vector for kinematic propagation
  // This the stuff inside the p function in eq. 47a - Mahony Paper
  turbomath::Vector wfinal = wbar - bias_ + kp * w_err;

  // Propagate Dynamics (only if we've moved)
  float sqrd_norm_w = wfinal.sqrd_norm();
  if (sqrd_norm_w > 0.0f)
  {
    float p = wfinal.x;
    float q = wfinal.y;
    float r = wfinal.z;

    if (RF_.params_.get_param_int(PARAM_FILTER_USE_MAT_EXP))
    {
      // Matrix Exponential Approximation (From Attitude Representation and Kinematic
      // Propagation for Low-Cost UAVs by Robert T. Casey)
      // (Eq. 12 Casey Paper)
      // This adds 90 us on STM32F10x chips
      float norm_w = sqrtf(sqrd_norm_w);
      turbomath::Quaternion qhat_np1;
      float t1 = cosf((norm_w*dt)/2.0f);
      float t2 = 1.0f/norm_w * sinf((norm_w*dt)/2.0f);
      qhat_np1.w = t1*state_.attitude.w + t2*(-p*state_.attitude.x - q*state_.attitude.y - r*state_.attitude.z);
      qhat_np1.x = t1*state_.attitude.x + t2*(p*state_.attitude.w + r*state_.attitude.y - q*state_.attitude.z);
      qhat_np1.y = t1*state_.attitude.y + t2*(q*state_.attitude.w - r*state_.attitude.x + p*state_.attitude.z);
      qhat_np1.z = t1*state_.attitude.z + t2*(r*state_.attitude.w + q*state_.attitude.x - p*state_.attitude.y);
      state_.attitude = qhat_np1.normalize();
    }
    else
    {
      // Euler Integration
      // (Eq. 47a Mahony Paper), but this is pretty straight-forward
      turbomath::Quaternion qdot(0.5f * (-p*state_.attitude.x - q*state_.attitude.y - r*state_.attitude.z),
                                 0.5f * (p*state_.attitude.w + r*state_.attitude.y - q*state_.attitude.z),
                                 0.5f * (q*state_.attitude.w - r*state_.attitude.x + p*state_.attitude.z),
                                 0.5f * (r*state_.attitude.w + q*state_.attitude.x - p*state_.attitude.y));
      state_.attitude.w += qdot.w*dt;
      state_.attitude.x += qdot.x*dt;
      state_.attitude.y += qdot.y*dt;
      state_.attitude.z += qdot.z*dt;
      state_.attitude.normalize();
    }
  }

  // Extract Euler Angles for controller
  state_.attitude.get_RPY(&state_.roll, &state_.pitch, &state_.yaw);

  // Save off adjust gyro measurements with estimated biases for control
  state_.angular_velocity = gyro_LPF_ - bias_;

  // If it has been more than 0.5 seconds since the acc update ran and we are supposed to be getting them
  // then trigger an unhealthy estimator error
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC) && now_us > 500000 + last_acc_update_us_
      && !RF_.params_.get_param_int(PARAM_FIXED_WING))
  {
    RF_.state_manager_.set_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
  else
  {
    RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
}

} // namespace rosflight_firmware
