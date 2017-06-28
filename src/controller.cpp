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

#include <stdint.h>
#include <stdbool.h>

#include "controller.h"
#include "rosflight.h"

namespace rosflight_firmware
{

Controller::Controller(ROSflight& rf) :
  RF_(rf)
{
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_ROLL_ANGLE_P);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_ROLL_ANGLE_I);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_ROLL_ANGLE_D);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_ROLL_RATE_P);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_ROLL_RATE_I);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_ROLL_RATE_D);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_PITCH_ANGLE_P);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_PITCH_ANGLE_I);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_PITCH_ANGLE_D);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_PITCH_RATE_P);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_PITCH_RATE_I);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_PITCH_RATE_D);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_YAW_RATE_P);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_YAW_RATE_I);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_YAW_RATE_D);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_MAX_COMMAND);
  RF_.params_.add_callback(std::bind(&Controller::param_change_callback, this, std::placeholders::_1), PARAM_PID_TAU);
}

void Controller::init()
{
  prev_time = 0.0f;

  float max = RF_.params_.get_param_float(PARAM_MAX_COMMAND);
  float min = -max;
  float tau = RF_.params_.get_param_float(PARAM_PID_TAU);

  roll_.init(RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_P),
             RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_I),
             RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_D),
             max, min, tau);
  roll_rate_.init(RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_P),
                  RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_I),
                  RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_D),
                  max, min, tau);
  pitch_.init(RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_P),
              RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_I),
              RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_D),
              max, min, tau);
  pitch_rate_.init(RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_P),
                   RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_I),
                   RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_D),
                   max, min, tau);
  yaw_rate_.init(RF_.params_.get_param_float(PARAM_PID_YAW_RATE_P),
                 RF_.params_.get_param_float(PARAM_PID_YAW_RATE_I),
                 RF_.params_.get_param_float(PARAM_PID_YAW_RATE_D),
                 max, min, tau);
}


void Controller::run()
{
  // Time calculation
  if (prev_time < 0.0000001)
  {
    prev_time = RF_.estimator_.get_estimator_timestamp() * 1e-6;
    return;
  }

  float now = RF_.estimator_.get_estimator_timestamp() * 1e-6;
  float dt = now - prev_time;
  prev_time = now;

  // Check if integrators should be updated
  //! @todo better way to figure out if throttle is high
  bool update_integrators = (RF_.state_manager_.state().armed) && (RF_.command_manager_._combined_control.F.value > 0.1f) && dt < 0.01f;

  // Based on the control types coming from the command manager, run the appropriate PID loops

  // ROLL
  if (RF_.command_manager_._combined_control.x.type == RATE)
    output_.x = roll_rate_.run(dt, RF_.estimator_.get_angular_velocity().x, RF_.command_manager_._combined_control.x.value, update_integrators);
  else if (RF_.command_manager_._combined_control.x.type == ANGLE)
    output_.x = roll_.run(dt, RF_.estimator_.get_roll(), RF_.command_manager_._combined_control.x.value, update_integrators, RF_.estimator_.get_angular_velocity().x);
  else
    output_.x = RF_.command_manager_._combined_control.x.value;

  // PITCH
  if (RF_.command_manager_._combined_control.y.type == RATE)
    output_.y = pitch_rate_.run(dt, RF_.estimator_.get_angular_velocity().y, RF_.command_manager_._combined_control.y.value, update_integrators);
  else if (RF_.command_manager_._combined_control.y.type == ANGLE)
    output_.y = pitch_.run(dt, RF_.estimator_.get_pitch(), RF_.command_manager_._combined_control.y.value, update_integrators, RF_.estimator_.get_angular_velocity().y);
  else
    output_.y = RF_.command_manager_._combined_control.y.value;

  // YAW
  if (RF_.command_manager_._combined_control.z.type == RATE)
    output_.z = yaw_rate_.run(dt, RF_.estimator_.get_angular_velocity().z, RF_.command_manager_._combined_control.z.value, update_integrators);
  else// PASSTHROUGH
    output_.z = RF_.command_manager_._combined_control.z.value;

  // Add feedforward torques
  output_.x += RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.y += RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.z += RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);
  output_.F = RF_.command_manager_._combined_control.F.value;
}

void Controller::calculate_equilbrium_torque_from_rc()
{
  // Make sure we are disarmed
  if (!(RF_.state_manager_.state().armed))
  {
    // Tell the user that we are doing a equilibrium torque calibration
    //    mavlink_log_warning("Capturing equilbrium offsets from RC");

    // Prepare for calibration
    // artificially tell the flight controller it is leveled
    // and zero out previously calculate offset torques
    RF_.estimator_.reset_state();

    RF_.params_.set_param_float(PARAM_X_EQ_TORQUE, 0.0);
    RF_.params_.set_param_float(PARAM_Y_EQ_TORQUE, 0.0);
    RF_.params_.set_param_float(PARAM_Z_EQ_TORQUE, 0.0);

    // pass the rc_control through the controller
    RF_.command_manager_._combined_control.x = RF_.command_manager_._rc_control.x;
    RF_.command_manager_._combined_control.y = RF_.command_manager_._rc_control.y;
    RF_.command_manager_._combined_control.z = RF_.command_manager_._rc_control.z;

    // dt is zero, so what this really does is applies the P gain with the settings
    // your RC transmitter, which if it flies level is a really good guess for
    // the static offset torques
    run();

    // the output from the controller is going to be the static offsets
    RF_.params_.set_param_float(PARAM_X_EQ_TORQUE, output_.x);
    RF_.params_.set_param_float(PARAM_Y_EQ_TORQUE, output_.y);
    RF_.params_.set_param_float(PARAM_Z_EQ_TORQUE, output_.z);

    //    mavlink_log_warning("Equilibrium torques found and applied.");
    //    mavlink_log_warning("Please zero out trims on your transmitter");
  }
  else
  {
    //    mavlink_log_warning("Cannot perform equilbirum offset calibration while armed");
  }
}

void Controller::param_change_callback(uint16_t param_id)
{
  init();
}

Controller::PID::PID() :
  kp_(0.0f),
  ki_(0.0f),
  kd_(0.0f),
  max_(1.0f),
  min_(-1.0f),
  integrator_(0.0f),
  differentiator_(0.0f),
  prev_x_(0.0f),
  tau_(0.05)
{}

void Controller::PID::init(float kp, float ki, float kd, float max, float min, float tau)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  max_ = max;
  min_ = min;
  tau_ = tau;
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator)
{
  float xdot;
  if (dt > 0.0001f)
  {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
    // The dirty derivative is a sort of low-pass filtered version of the derivative.
    //// (Include reference to Dr. Beard's notes here)
    differentiator_ = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
                          + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
    xdot = differentiator_;
  }
  else
  {
    xdot = 0.0f;
  }
  prev_x_ = x;

  return run(dt, x, x_c, update_integrator, xdot);
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator, float xdot)
{
  // Calculate Error (make sure to de-reference pointers)
  float error = x_c - x;

  // Initialize Terms
  float p_term = error * kp_;
  float i_term = 0.0f;
  float d_term = 0.0f;

  // If there is a derivative term
  if (kd_ > 0.0f)
  {
    d_term = kd_ * xdot;
  }

  // If there is an integrator term and we are updating integrators
  if ((ki_ > 0.0f) && update_integrator)
  {
      // integrate
      integrator_ += error * dt;
      // calculate I term
      i_term = ki_ * integrator_;
  }

  // sum three terms
  float u = p_term + i_term - d_term;

  // Integrator anti-windup
  //// Include reference to Dr. Beard's notes here
  float u_sat = (u > max_) ? max_ : (u < min_) ? min_ : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > 0.0f)
    integrator_ = (u_sat - p_term + d_term)/ki_;

  // Set output
  return u_sat;
}

} // namespace rosflight_firmware
