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
  q_.w = 1.0f;
  q_.x = 0.0f;
  q_.y = 0.0f;
  q_.z = 0.0f;
  omega_.x = 0.0f;
  omega_.y = 0.0f;
  omega_.z = 0.0f;
  roll_ = 0.0f;
  pitch_ = 0.0f;
  yaw_ = 0.0f;

  q_hat_.w = 1.0f;
  q_hat_.x = 0.0f;
  q_hat_.y = 0.0f;
  q_hat_.z = 0.0f;

  w1_.x = 0.0f;
  w1_.y = 0.0f;
  w1_.z = 0.0f;

  w2_.x = 0.0f;
  w2_.y = 0.0f;
  w2_.z = 0.0f;

  bias_.x = 0.0f;
  bias_.y = 0.0f;
  bias_.z = 0.0f;

  w_acc_.x = 0.0f;
  w_acc_.y = 0.0f;
  w_acc_.z = 0.0f;

  q_tilde_.w = 1.0f;
  q_tilde_.x = 0.0f;
  q_tilde_.y = 0.0f;
  q_tilde_.z = 0.0f;

  accel_LPF_.x = 0;
  accel_LPF_.y = 0;
  accel_LPF_.z = -9.80665;

  gyro_LPF_.x = 0;
  gyro_LPF_.y = 0;
  gyro_LPF_.z = 0;

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
  reset_state();
}

void Estimator::run_LPF()
{
  float alpha_acc = RF_.params_.get_param_float(PARAM_ACC_ALPHA);
  vector_t raw_accel = RF_.sensors_.data().accel;
  accel_LPF_.x = (1.0f-alpha_acc)*raw_accel.x + alpha_acc*accel_LPF_.x;
  accel_LPF_.y = (1.0f-alpha_acc)*raw_accel.y + alpha_acc*accel_LPF_.y;
  accel_LPF_.z = (1.0f-alpha_acc)*raw_accel.z + alpha_acc*accel_LPF_.z;

  float alpha_gyro = RF_.params_.get_param_float(PARAM_GYRO_ALPHA);
  vector_t raw_gyro = RF_.sensors_.data().gyro;
  gyro_LPF_.x = (1.0f-alpha_gyro)*raw_gyro.x + alpha_gyro*gyro_LPF_.x;
  gyro_LPF_.y = (1.0f-alpha_gyro)*raw_gyro.y + alpha_gyro*gyro_LPF_.y;
  gyro_LPF_.z = (1.0f-alpha_gyro)*raw_gyro.z + alpha_gyro*gyro_LPF_.z;
}


void Estimator::run_estimator()
{
  static float kp, ki;
  now_us_ = RF_.sensors_.data().imu_time;
  if (last_time_ == 0)
  {
    last_time_ = now_us_;
    last_acc_update_us_ = last_time_;
    return;
  }
  else if (now_us_ <= last_time_)
  {
    // this shouldn't happen
//    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    last_time_ = now_us_;
    return;
  }

//  RF_.state_manager_.clear_error(StateManager::ERROR_TIME_GOING_BACKWARDS);

  float dt = (now_us_ - last_time_) * 1e-6f;
  last_time_ = now_us_;

  // Crank up the gains for the first few seconds for quick convergence
  if (now_us_ < (uint64_t)RF_.params_.get_param_int(PARAM_INIT_TIME)*1000)
  {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP)*10.0f;
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI)*10.0f;
  }
  else
  {
    kp = RF_.params_.get_param_float(PARAM_FILTER_KP);
    ki = RF_.params_.get_param_float(PARAM_FILTER_KI);
  }

  // Run LPF to reject a lot of noise
  run_LPF();

  // add in accelerometer
  float a_sqrd_norm = accel_LPF_.x*accel_LPF_.x + accel_LPF_.y*accel_LPF_.y + accel_LPF_.z*accel_LPF_.z;

  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC)
      && a_sqrd_norm < 1.15f*1.15f*9.80665f*9.80665f && a_sqrd_norm > 0.85f*0.85f*9.80665f*9.80665f)
  {
    last_acc_update_us_ = now_us_;
    // Get error estimated by accelerometer measurement
    vector_t a = vector_normalize(accel_LPF_);
    // Get the quaternion from accelerometer (low-frequency measure q)
    // (Not in either paper)
    quaternion_t q_acc_inv = quaternion_inverse(quat_from_two_vectors(a, g_));
    // Get the error quaternion between observer and low-freq q
    // Below Eq. 45 Mahony Paper
    q_tilde_ = quaternion_multiply(q_acc_inv, q_hat_);
    // Correction Term of Eq. 47a and 47b Mahony Paper
    // w_acc = 2*s_tilde*v_tilde
    w_acc_.x = -2.0f*q_tilde_.w*q_tilde_.x;
    w_acc_.y = -2.0f*q_tilde_.w*q_tilde_.y;
    w_acc_.z = 0.0f; // Don't correct z, because it's unobservable from the accelerometer

    // integrate biases from accelerometer feedback
    // (eq 47b Mahony Paper, using correction term w_acc found above)
    bias_.x -= ki*w_acc_.x*dt;
    bias_.y -= ki*w_acc_.y*dt;
    bias_.z = 0.0;  // Don't integrate z bias, because it's unobservable
  }
  else
  {
    w_acc_.x = 0.0f;
    w_acc_.y = 0.0f;
    w_acc_.z = 0.0f;
  }

  // Pull out Gyro measurements
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_QUAD_INT))
  {
    // Quadratic Integration (Eq. 14 Casey Paper)
    // this integration step adds 12 us on the STM32F10x chips
    wbar_ = vector_add(vector_add(scalar_multiply(-1.0f/12.0f,w2_), scalar_multiply(8.0f/12.0f,w1_)),
                      scalar_multiply(5.0f/12.0f,gyro_LPF_));
    w2_ = w1_;
    w1_ = gyro_LPF_;
  }
  else
  {
    wbar_ = gyro_LPF_;
  }

  // Build the composite omega vector for kinematic propagation
  // This the stuff inside the p function in eq. 47a - Mahony Paper
  wfinal_ = vector_add(vector_sub(wbar_, bias_), scalar_multiply(kp, w_acc_));

  // Propagate Dynamics (only if we've moved)
  float sqrd_norm_w = sqrd_norm(wfinal_);
  if (sqrd_norm_w > 0.0f)
  {
    float p = wfinal_.x;
    float q = wfinal_.y;
    float r = wfinal_.z;

    if (RF_.params_.get_param_int(PARAM_FILTER_USE_MAT_EXP))
    {
      // Matrix Exponential Approximation (From Attitude Representation and Kinematic
      // Propagation for Low-Cost UAVs by Robert T. Casey)
      // (Eq. 12 Casey Paper)
      // This adds 90 us on STM32F10x chips
      float norm_w = sqrt(sqrd_norm_w);
      quaternion_t qhat_np1;
      float t1 = cos((norm_w*dt)/2.0f);
      float t2 = 1.0f/norm_w * sin((norm_w*dt)/2.0f);
      qhat_np1.w = t1*q_hat_.w   + t2*(- p*q_hat_.x - q*q_hat_.y - r*q_hat_.z);
      qhat_np1.x = t1*q_hat_.x   + t2*(p*q_hat_.w             + r*q_hat_.y - q*q_hat_.z);
      qhat_np1.y = t1*q_hat_.y   + t2*(q*q_hat_.w - r*q_hat_.x             + p*q_hat_.z);
      qhat_np1.z = t1*q_hat_.z   + t2*(r*q_hat_.w + q*q_hat_.x - p*q_hat_.y);
      q_hat_ = quaternion_normalize(qhat_np1);
    }
    else
    {
      // Euler Integration
      // (Eq. 47a Mahony Paper), but this is pretty straight-forward
      quaternion_t qdot = {0.5f * (- p*q_hat_.x - q*q_hat_.y - r*q_hat_.z),
                           0.5f * (p*q_hat_.w             + r*q_hat_.y - q*q_hat_.z),
                           0.5f * (q*q_hat_.w - r*q_hat_.x             + p*q_hat_.z),
                           0.5f * (r*q_hat_.w + q*q_hat_.x - p*q_hat_.y)
                          };
      q_hat_.w += qdot.w*dt;
      q_hat_.x += qdot.x*dt;
      q_hat_.y += qdot.y*dt;
      q_hat_.z += qdot.z*dt;
      q_hat_ = quaternion_normalize(q_hat_);
    }
  }

  // Save attitude estimate
  q_ = q_hat_;

  // Extract Euler Angles for controller
  euler_from_quat(q_, &roll_, &pitch_, &yaw_);

  // Save off adjust gyro measurements with estimated biases for control
  omega_ = vector_sub(gyro_LPF_, bias_);

  // If it has been more than 0.5 seconds since the acc update ran and we are supposed to be getting them
  // then trigger an unhealthy estimator error
  if (RF_.params_.get_param_int(PARAM_FILTER_USE_ACC) && now_us_ > 500000 + last_acc_update_us_)
  {
    RF_.state_manager_.set_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
  else
  {
    RF_.state_manager_.clear_error(StateManager::ERROR_UNHEALTHY_ESTIMATOR);
  }
}

}
