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

namespace rosflight
{


void Controller::init_pid(pid_t *pid, uint16_t kp_param_id, uint16_t ki_param_id,
                          uint16_t kd_param_id, float *current_x, float *current_xdot,
                          float *commanded_x, float *output, float max, float min)
{
  pid->kp_param_id = kp_param_id;
  pid->ki_param_id = ki_param_id;
  pid->kd_param_id = kd_param_id;
  pid->current_x = current_x;
  pid->current_xdot = current_xdot;
  pid->commanded_x = commanded_x;
  pid->output = output;
  pid->max = max;
  pid->min = min;
  pid->integrator = 0.0;
  pid->differentiator = 0.0;
  pid->prev_x = 0.0;
  pid->tau = params->get_param_float(PARAM_PID_TAU);
}


void Controller::run_pid(pid_t *pid, float dt)
{
  if (dt > 0.010 || !fsm->armed())
  {
    // This means that this is a ''stale'' controller and needs to be reset.
    // This would happen if we have been operating in a different mode for a while
    // and will result in some enormous integrator, and weird behavior on the derivative term.
    // Obiously, it could also mean we are disarmed and shouldn't integrate
    // Setting dt to zero for this loop will mean that the integrator and dirty derivative
    // doesn't do anything this time but will keep it from exploding.
    dt = 0.0;
    pid->differentiator = 0.0;
  }

  // Calculate Error (make sure to de-reference pointers)
  float error = (*pid->commanded_x) - (*pid->current_x);

  // Initialize Terms
  float p_term = error * params->get_param_float(pid->kp_param_id);
  float i_term = 0.0;
  float d_term = 0.0;

  // If there is a derivative term
  if (pid->kd_param_id < PARAMS_COUNT)
  {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
    // The dirty derivative is a sort of low-pass filtered version of the derivative.
    // (Be sure to de-reference pointers)
    if (pid->current_xdot == NULL)
    {
      if (dt > 0.0f)
      {
        pid->differentiator = (2.0f*pid->tau-dt)/(2.0f*pid->tau+dt)*pid->differentiator
            + 2.0f/(2.0f*pid->tau+dt)*((*pid->current_x) - pid->prev_x);
        pid->prev_x = *pid->current_x;
        d_term = params->get_param_float(pid->kd_param_id) * pid->differentiator;
      }
    }
    else
    {
      d_term = params->get_param_float(pid->kd_param_id) * (*pid->current_xdot);
    }
  }

  // If there is an integrator, we are armed, and throttle is high
  /// TODO: better way to figure out if throttle is high
  if ((pid->ki_param_id < PARAMS_COUNT) && (fsm->armed()) && (mux->_combined_control.F.value > 0.1))
  {
    if (params->get_param_float(pid->ki_param_id) > 0.0)
    {
      // integrate
      pid->integrator += error*dt;
      // calculate I term (be sure to de-reference pointer to gain)
      i_term = params->get_param_float(pid->ki_param_id) * pid->integrator;
    }
  }

  // sum three terms
  float u = p_term + i_term - d_term;

  // Integrator anti-windup
  float u_sat = (u > pid->max) ? pid->max : (u < pid->min) ? pid->min : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term))
    pid->integrator = (u_sat - p_term + d_term)/params->get_param_float(pid->ki_param_id);

  // Set output
  (*pid->output) = u_sat;

  return;
}


void Controller::init_controller(Mode *_fsm, Board *_board, //Mux* _mux, Mixer* _mixer,
                                 Estimator *_estimator, Params *_params)
{
  fsm = _fsm;
  board = _board;
  estimator = _estimator;
  params = _params;

  prev_time = 0.0f;

  init_pid(&pid_roll,
           PARAM_PID_ROLL_ANGLE_P,
           PARAM_PID_ROLL_ANGLE_I,
           PARAM_PID_ROLL_ANGLE_D,
           &estimator->roll,
           &estimator->omega.x,
           &mux->_combined_control.x.value,
           &mixer->_command.x,
           params->get_param_float(PARAM_MAX_COMMAND),
           -1.0f*params->get_param_float(PARAM_MAX_COMMAND));

  init_pid(&pid_pitch,
           PARAM_PID_PITCH_ANGLE_P,
           PARAM_PID_PITCH_ANGLE_I,
           PARAM_PID_PITCH_ANGLE_D,
           &estimator->pitch,
           &estimator->omega.y,
           &mux->_combined_control.y.value,
           &mixer->_command.y,
           params->get_param_float(PARAM_MAX_COMMAND),
           -1.0f*params->get_param_float(PARAM_MAX_COMMAND));

  init_pid(&pid_roll_rate,
           PARAM_PID_ROLL_RATE_P,
           PARAM_PID_ROLL_RATE_I,
           PARAM_PID_ROLL_RATE_D,
           &estimator->omega.x,
           NULL,
           &mux->_combined_control.x.value,
           &mixer->_command.x,
           params->get_param_float(PARAM_MAX_COMMAND),
           -1.0f*params->get_param_float(PARAM_MAX_COMMAND));

  init_pid(&pid_pitch_rate,
           PARAM_PID_PITCH_RATE_P,
           PARAM_PID_PITCH_RATE_I,
           PARAM_PID_PITCH_RATE_D,
           &estimator->omega.y,
           NULL,
           &mux->_combined_control.y.value,
           &mixer->_command.y,
           params->get_param_float(PARAM_MAX_COMMAND),
           -1.0f*params->get_param_float(PARAM_MAX_COMMAND));

  init_pid(&pid_yaw_rate,
           PARAM_PID_YAW_RATE_P,
           PARAM_PID_YAW_RATE_I,
           PARAM_PID_YAW_RATE_D,
           &estimator->omega.z,
           NULL,
           &mux->_combined_control.z.value,
           &mixer->_command.z,
           params->get_param_float(PARAM_MAX_COMMAND),
           -1.0f*params->get_param_float(PARAM_MAX_COMMAND));
}


void Controller::run_controller()
{
  // Time calculation
  if (prev_time < 0.0000001)
  {
    prev_time = estimator->get_estimator_timestamp() * 1e-6;
    return;
  }

  float now = estimator->get_estimator_timestamp() * 1e-6;
  float dt = now - prev_time;
  prev_time = now;

  // ROLL
  if (mux->_combined_control.x.type == RATE)
    run_pid(&pid_roll_rate, dt);
  else if (mux->_combined_control.x.type == ANGLE)
    run_pid(&pid_roll, dt);
  else // PASSTHROUGH
    mixer->_command.x = mux->_combined_control.x.value;

  // PITCH
  if (mux->_combined_control.y.type == RATE)
    run_pid(&pid_pitch_rate, dt);
  else if (mux->_combined_control.y.type == ANGLE)
    run_pid(&pid_pitch, dt);
  else // PASSTHROUGH
    mixer->_command.y = mux->_combined_control.y.value;

  // YAW
  if (mux->_combined_control.z.type == RATE)
    run_pid(&pid_yaw_rate, dt);
  else// PASSTHROUGH
    mixer->_command.z = mux->_combined_control.z.value;

  // Add feedforward torques
  mixer->_command.x += params->get_param_float(PARAM_X_EQ_TORQUE);
  mixer->_command.y += params->get_param_float(PARAM_Y_EQ_TORQUE);
  mixer->_command.z += params->get_param_float(PARAM_Z_EQ_TORQUE);
  mixer->_command.F = mux->_combined_control.F.value;
}

void Controller::calculate_equilbrium_torque_from_rc()
{
  // Make sure we are disarmed
  if (!(fsm->armed()))
  {
    // Tell the user that we are doing a equilibrium torque calibration
    //    mavlink_log_warning("Capturing equilbrium offsets from RC");

    // Prepare for calibration
    // artificially tell the flight controller it is leveled
    // and zero out previously calculate offset torques
    estimator->omega.x = 0.0;
    estimator->omega.y = 0.0;
    estimator->omega.z = 0.0;
    estimator->q.w = 1.0;
    estimator->q.x = 0.0;
    estimator->q.y = 0.0;
    estimator->q.z = 0.0;

    params->set_param_float(PARAM_X_EQ_TORQUE, 0.0);
    params->set_param_float(PARAM_Y_EQ_TORQUE, 0.0);
    params->set_param_float(PARAM_Z_EQ_TORQUE, 0.0);

    // pass the rc_control through the controller
    mux->_combined_control.x = mux->_rc_control.x;
    mux->_combined_control.y = mux->_rc_control.y;
    mux->_combined_control.z = mux->_rc_control.z;

    run_controller();

    // the output from the controller is going to be the static offsets
    params->set_param_float(PARAM_X_EQ_TORQUE, mixer->_command.x);
    params->set_param_float(PARAM_Y_EQ_TORQUE, mixer->_command.y);
    params->set_param_float(PARAM_Z_EQ_TORQUE, mixer->_command.z);

    //    mavlink_log_warning("Equilibrium torques found and applied.");
    //    mavlink_log_warning("Please zero out trims on your transmitter");
  }
  else
  {
    //    mavlink_log_warning("Cannot perform equilbirum offset calibration while armed");
  }
}

}
