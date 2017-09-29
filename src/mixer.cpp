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

#include "mixer.h"
#include "rosflight.h"

namespace rosflight_firmware
{

Mixer::Mixer(ROSflight &_rf) :
  RF_(_rf)
{}

void Mixer::init()
{
  RF_.params_.add_callback(std::bind(&Mixer::param_change_callback, this, std::placeholders::_1), PARAM_MOTOR_PWM_SEND_RATE);
  RF_.params_.add_callback(std::bind(&Mixer::param_change_callback, this, std::placeholders::_1), PARAM_MOTOR_MIN_PWM);
  RF_.params_.add_callback(std::bind(&Mixer::param_change_callback, this, std::placeholders::_1), PARAM_RC_TYPE);
  RF_.params_.add_callback(std::bind(&Mixer::param_change_callback, this, std::placeholders::_1), PARAM_MIXER);

  init_mixing();
  init_PWM();
}

void Mixer::param_change_callback(uint16_t param_id)
{
  switch (param_id)
  {
  case PARAM_MIXER:
    init_mixing();
    break;
  default:
    init_PWM();
    break;
  }
}


void Mixer::init_mixing()
{
  // clear the invalid mixer error
  RF_.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);

  uint8_t mixer_choice = RF_.params_.get_param_int(PARAM_MIXER);

  if (mixer_choice >= NUM_MIXERS)
  {
    RF_.comm_manager_.log(CommLink::LogSeverity::LOG_ERROR, "Invalid Mixer Choice");
    mixer_choice = 0;

    // set the invalid mixer flag
    RF_.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
  }

  mixer_to_use_ = array_of_mixers_[mixer_choice];

  for (int8_t i=0; i<8; i++)
  {
    raw_outputs_[i] = 0.0f;
    unsaturated_outputs_[i] = 0.0f;
  }
}

void Mixer::init_PWM()
{
  bool useCPPM = false;
  if (RF_.params_.get_param_int(PARAM_RC_TYPE) == 1)
  {
    useCPPM = true;
  }
  int16_t motor_refresh_rate = RF_.params_.get_param_int(PARAM_MOTOR_PWM_SEND_RATE);
  int16_t off_pwm = RF_.params_.get_param_int(PARAM_MOTOR_MIN_PWM);
  RF_.board_.pwm_init(useCPPM, motor_refresh_rate, off_pwm);
}


void Mixer::write_motor(uint8_t index, float value)
{
  if (RF_.state_manager_.state().armed)
  {
    if (value > 1.0)
    {
      value = 1.0;
    }
    else if (value < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)
             && RF_.params_.get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED))
    {
      value = RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE);
    }
    else if (value < 0.0)
    {
      value = 0.0;
    }
  }
  else
  {
    value = 0.0;
  }
  raw_outputs_[index] = value;
  int32_t pwm_us = value * (RF_.params_.get_param_int(PARAM_MOTOR_MAX_PWM) - RF_.params_.get_param_int(
                              PARAM_MOTOR_MIN_PWM)) + RF_.params_.get_param_int(PARAM_MOTOR_MIN_PWM);
  RF_.board_.pwm_write(index, pwm_us);
}


void Mixer::write_servo(uint8_t index, float value)
{
  if (value > 1.0)
  {
    value = 1.0;
  }
  else if (value < -1.0)
  {
    value = -1.0;
  }
  raw_outputs_[index] = value;
  RF_.board_.pwm_write(index, raw_outputs_[index] * 500 + 1500);
}


void Mixer::mix_output()
{
  Controller::Output commands = RF_.controller_.output();
  float max_output = 1.0f;

  // Reverse Fixedwing channels just before mixing if we need to
  if (RF_.params_.get_param_int(PARAM_FIXED_WING))
  {
    commands.x *= RF_.params_.get_param_int(PARAM_AILERON_REVERSE) ? -1 : 1;
    commands.y *= RF_.params_.get_param_int(PARAM_ELEVATOR_REVERSE) ? -1 : 1;
    commands.z *= RF_.params_.get_param_int(PARAM_RUDDER_REVERSE) ? -1 : 1;
  }

  for (int8_t i=0; i<8; i++)
  {
    if (mixer_to_use_->output_type[i] != NONE)
    {
      // Matrix multiply to mix outputs
      unsaturated_outputs_[i] = (commands.F*mixer_to_use_->F[i] + commands.x*mixer_to_use_->x[i] +
                              commands.y*mixer_to_use_->y[i] + commands.z*mixer_to_use_->z[i]);

      // Save off the largest control output if it is greater than 1.0 for future scaling
      if (unsaturated_outputs_[i] > max_output)
      {
        max_output = unsaturated_outputs_[i];
      }
    }
  }

  // saturate outputs to maintain controllability even during aggressive maneuvers
  float scale_factor = 1.0;
  if (max_output > 1.0)
  {
    scale_factor = 1.0/max_output;
  }



  for (int8_t i=0; i<8; i++)
  {
    // Write output to motors
    if (mixer_to_use_->output_type[i] == S)
    {
      write_servo(i, unsaturated_outputs_[i]);
    }
    else if (mixer_to_use_->output_type[i] == M)
    {
      // scale all motor outputs by scale factor (this is usually 1.0, unless we saturated)
      unsaturated_outputs_[i] *= scale_factor;
      write_motor(i, unsaturated_outputs_[i]);
    }
  }
}

}
