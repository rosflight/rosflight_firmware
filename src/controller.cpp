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

#include "controller.h"

#include "command_manager.h"
#include "estimator.h"

#include "rosflight.h"

#include <cstdbool>
#include <cstdint>

namespace rosflight_firmware
{
Controller::Controller(ROSflight & rf)
    : RF_(rf)
{}

void Controller::init()
{
  prev_time_us_ = 0;

  // Calculate the max thrust to convert from throttle setting to thrust
  calculate_max_thrust();

  // Don't saturate the torque values
  float max_torque = INFINITY;
  float min_torque = -max_torque;

  float tau = RF_.params_.get_param_float(PARAM_PID_TAU);

  roll_.init(RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_P),
             RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_I),
             RF_.params_.get_param_float(PARAM_PID_ROLL_ANGLE_D), max_torque, min_torque, tau);
  roll_rate_.init(RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_P),
                  RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_I),
                  RF_.params_.get_param_float(PARAM_PID_ROLL_RATE_D), max_torque, min_torque, tau);
  pitch_.init(RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_P),
              RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_I),
              RF_.params_.get_param_float(PARAM_PID_PITCH_ANGLE_D), max_torque, min_torque, tau);
  pitch_rate_.init(RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_P),
                   RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_I),
                   RF_.params_.get_param_float(PARAM_PID_PITCH_RATE_D), max_torque, min_torque, tau);
  yaw_rate_.init(RF_.params_.get_param_float(PARAM_PID_YAW_RATE_P),
                 RF_.params_.get_param_float(PARAM_PID_YAW_RATE_I),
                 RF_.params_.get_param_float(PARAM_PID_YAW_RATE_D), max_torque, min_torque, tau);
}

void Controller::calculate_max_thrust()
{
  float R = RF_.params_.get_param_float(PARAM_MOTOR_RESISTANCE);
  float D = RF_.params_.get_param_float(PARAM_PROP_DIAMETER);
  float rho = RF_.params_.get_param_float(PARAM_AIR_DENSITY);
  float CQ = RF_.params_.get_param_float(PARAM_PROP_CQ);
  float CT = RF_.params_.get_param_float(PARAM_PROP_CT);
  float KV = RF_.params_.get_param_float(PARAM_MOTOR_KV);
  float i0 = RF_.params_.get_param_float(PARAM_NO_LOAD_CURRENT);
  float KQ = KV;
  int num_motors = RF_.params_.get_param_int(PARAM_NUM_MOTORS);

  float V_max = RF_.params_.get_param_float(PARAM_VOLT_MAX);
  float a = R * rho * pow(D, 5.0) * CQ / (4 * pow(M_PI, 2.0) * KQ);
  float b = KV;
  float c = i0 * R - V_max;

  // Using Eq. 4.19 and setting equal to the equation at the beginning of 4.3 in Small Unmanned Aircraft
  // We only need the positive root. Since a and b positive, sqrt term must be positive
  float omega = (-b + sqrt(pow(b, 2.0) - 4 * a * c)) / (2 * a);

  // Calculate the max thrust from Eq in 4.3 of Small Unmanned Aircraft
  // Note that the equation is for a single motor, so to calculate max thrust, we need to multiply by the number of motors
  max_thrust_ = rho * pow(D, 4.0) * CT * pow(omega, 2.0) / (4 * pow(M_PI, 2.0)) * num_motors;
}

void Controller::run()
{
  // Time calculation
  if (prev_time_us_ == 0) {
    prev_time_us_ = RF_.estimator_.state().timestamp_us;
    return;
  }

  int32_t dt_us = (RF_.estimator_.state().timestamp_us - prev_time_us_);
  if (dt_us < 0) {
    RF_.state_manager_.set_error(StateManager::ERROR_TIME_GOING_BACKWARDS);
    return;
  }
  prev_time_us_ = RF_.estimator_.state().timestamp_us;

  // Check if integrators should be updated
  //! @todo better way to figure out if throttle is high
  bool update_integrators = (RF_.state_manager_.state().armed)
    && (RF_.command_manager_.combined_control().Fz.value > 0.1f) && dt_us < 10000;

  // Run the PID loops
  Controller::Output pid_output = run_pid_loops(
    dt_us, RF_.estimator_.state(), RF_.command_manager_.combined_control(), update_integrators);

  // Add feedforward torques
  // Note that the controller does not alter Fx and Fy
  output_.Qx = pid_output.Qx + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE);
  output_.Qy = pid_output.Qy + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE);
  output_.Qz = pid_output.Qz + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE);
  output_.Fz = pid_output.Fz;
  output_.Fx = RF_.command_manager_.combined_control().Fx.value;
  output_.Fy = RF_.command_manager_.combined_control().Fy.value;
}

void Controller::calculate_equilbrium_torque_from_rc()
{
  // TODO: Verify that this is working!
  // Make sure we are disarmed
  if (!(RF_.state_manager_.state().armed)) {
    // Tell the user that we are doing a equilibrium torque calibration
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                          "Capturing equilbrium offsets from RC");

    // Prepare for calibration
    // artificially tell the flight controller it is leveled
    Estimator::State fake_state;
    fake_state.angular_velocity.x = 0.0f;
    fake_state.angular_velocity.y = 0.0f;
    fake_state.angular_velocity.z = 0.0f;

    fake_state.attitude.x = 0.0f;
    fake_state.attitude.y = 0.0f;
    fake_state.attitude.z = 0.0f;
    fake_state.attitude.w = 1.0f;

    fake_state.roll = 0.0f;
    fake_state.pitch = 0.0f;
    fake_state.yaw = 0.0f;

    // pass the rc_control through the controller
    // dt is zero, so what this really does is applies the P gain with the settings
    // your RC transmitter, which if it flies level is a really good guess for
    // the static offset torques
    Controller::Output pid_output =
      run_pid_loops(0, fake_state, RF_.command_manager_.rc_control(), false);

    // the output from the controller is going to be the static offsets
    RF_.params_.set_param_float(PARAM_X_EQ_TORQUE,
                                pid_output.Qx + RF_.params_.get_param_float(PARAM_X_EQ_TORQUE));
    RF_.params_.set_param_float(PARAM_Y_EQ_TORQUE,
                                pid_output.Qy + RF_.params_.get_param_float(PARAM_Y_EQ_TORQUE));
    RF_.params_.set_param_float(PARAM_Z_EQ_TORQUE,
                                pid_output.Qz + RF_.params_.get_param_float(PARAM_Z_EQ_TORQUE));

    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                          "Equilibrium torques found and applied.");
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                          "Please zero out trims on your transmitter");
  } else {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                          "Cannot perform equilibrium offset calibration while armed");
  }
}

void Controller::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_PID_ROLL_ANGLE_P:
    case PARAM_PID_ROLL_ANGLE_I:
    case PARAM_PID_ROLL_ANGLE_D:
    case PARAM_PID_ROLL_RATE_P:
    case PARAM_PID_ROLL_RATE_I:
    case PARAM_PID_ROLL_RATE_D:
    case PARAM_PID_PITCH_ANGLE_P:
    case PARAM_PID_PITCH_ANGLE_I:
    case PARAM_PID_PITCH_ANGLE_D:
    case PARAM_PID_PITCH_RATE_P:
    case PARAM_PID_PITCH_RATE_I:
    case PARAM_PID_PITCH_RATE_D:
    case PARAM_PID_YAW_RATE_P:
    case PARAM_PID_YAW_RATE_I:
    case PARAM_PID_YAW_RATE_D:
    case PARAM_MOTOR_RESISTANCE:
    case PARAM_PROP_DIAMETER:
    case PARAM_AIR_DENSITY:
    case PARAM_PROP_CQ:
    case PARAM_PROP_CT:
    case PARAM_MOTOR_KV:
    case PARAM_NO_LOAD_CURRENT:
    case PARAM_VOLT_MAX:
    case PARAM_NUM_MOTORS:
    case PARAM_RC_MAX_THROTTLE:
    case PARAM_PID_TAU:
      init();
      break;
    default:
      // do nothing
      break;
  }
}

Controller::Output Controller::run_pid_loops(uint32_t dt_us, const Estimator::State & state,
                                             const control_t & command, bool update_integrators)
{
  // Based on the control types coming from the command manager, run the appropriate PID loops
  Controller::Output out;

  float dt = 1e-6 * dt_us;

  // ROLL
  if (command.Qx.type == RATE) {
    out.Qx = roll_rate_.run(dt, state.angular_velocity.x, command.Qx.value, update_integrators);
  } else if (command.Qx.type == ANGLE) {
    out.Qx =
      roll_.run(dt, state.roll, command.Qx.value, update_integrators, state.angular_velocity.x);
  } else {
    out.Qx = command.Qx.value;
  }

  // PITCH
  if (command.Qy.type == RATE) {
    out.Qy = pitch_rate_.run(dt, state.angular_velocity.y, command.Qy.value, update_integrators);
  } else if (command.Qy.type == ANGLE) {
    out.Qy =
      pitch_.run(dt, state.pitch, command.Qy.value, update_integrators, state.angular_velocity.y);
  } else {
    out.Qy = command.Qy.value;
  }

  // YAW
  if (command.Qz.type == RATE) {
    out.Qz = yaw_rate_.run(dt, state.angular_velocity.z, command.Qz.value, update_integrators);
  } else {
    out.Qz = command.Qz.value;
  }

  // THROTTLE
  if (command.Fz.type == THROTTLE) {
    // Scales the saturation limit by RC_MAX_THROTTLE to maintain controllability 
    // during aggressive maneuvers.
    // Also note the negative sign. Since the mixer assumes the inputs are in the NED
    // frame, a throttle command corresponds to a thrust command in the negative direction.
    out.Fz = -command.Fz.value * RF_.params_.get_param_float(PARAM_RC_MAX_THROTTLE);

    if (RF_.mixer_.use_motor_parameters()) {
      out.Fz *= max_thrust_;
    }
  } else {
    // If it is not a throttle setting then pass directly to the mixer.
    out.Fz = command.Fz.value;
  }

  return out;
}

Controller::PID::PID()
    : kp_(0.0f)
    , ki_(0.0f)
    , kd_(0.0f)
    , max_(1.0f)
    , min_(-1.0f)
    , integrator_(0.0f)
    , differentiator_(0.0f)
    , prev_x_(0.0f)
    , tau_(0.05)
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
  if (dt > 0.0001f) {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the
    // derivative) The dirty derivative is a sort of low-pass filtered version of the derivative.
    // See Eq. 10.4 of Introduction to Feedback Control by Beard, McLain, Peterson, Killpack
    differentiator_ = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
      + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
    xdot = differentiator_;
  } else {
    xdot = 0.0f;
  }
  prev_x_ = x;

  return run(dt, x, x_c, update_integrator, xdot);
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator, float xdot)
{
  // Calculate Error
  float error = x_c - x;

  // Initialize Terms
  float p_term = error * kp_;
  float i_term = 0.0f;
  float d_term = 0.0f;

  // If there is a derivative term
  if (kd_ > 0.0f) { d_term = kd_ * xdot; }

  // If there is an integrator term and we are updating integrators
  if ((ki_ > 0.0f) && update_integrator) {
    // integrate
    integrator_ += error * dt;
    // calculate I term
    i_term = ki_ * integrator_;
  }

  // sum three terms
  float u = p_term - d_term + i_term;

  // Integrator anti-windup
  float u_sat = (u > max_) ? max_ : (u < min_) ? min_ : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > 0.0f) {
    integrator_ = (u_sat - p_term + d_term) / ki_;
  }

  // Set output
  return u_sat;
}

} // namespace rosflight_firmware
