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

#include "mixer.h"

#include "rosflight.h"

#include <cstdint>
#include <iostream>
// #include "eigen3/Eigen/Dense"

namespace rosflight_firmware
{
Mixer::Mixer(ROSflight & _rf)
    : RF_(_rf)
{
  mixer_to_use_ = nullptr;
}

void Mixer::init() { init_mixing(); }

void Mixer::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_MIXER:
    case PARAM_MOTOR_RESISTANCE:
    case PARAM_AIR_DENSITY:
    case PARAM_MOTOR_KV:
    case PARAM_NO_LOAD_CURRENT:
    case PARAM_PROP_DIAMETER:
    case PARAM_PROP_CT:
    case PARAM_PROP_CQ:
    case PARAM_NUM_MOTORS:
    case PARAM_VOLT_MAX:
    case PARAM_MOTOR_0_POS:
    case PARAM_MOTOR_1_POS:
    case PARAM_MOTOR_2_POS:
    case PARAM_MOTOR_3_POS:
    case PARAM_MOTOR_4_POS:
    case PARAM_MOTOR_5_POS:
    case PARAM_MOTOR_6_POS:
    case PARAM_MOTOR_7_POS:
    case PARAM_MOTOR_8_POS:
    case PARAM_MOTOR_9_POS:
    case PARAM_MOTOR_0_PSI:
    case PARAM_MOTOR_1_PSI:
    case PARAM_MOTOR_2_PSI:
    case PARAM_MOTOR_3_PSI:
    case PARAM_MOTOR_4_PSI:
    case PARAM_MOTOR_5_PSI:
    case PARAM_MOTOR_6_PSI:
    case PARAM_MOTOR_7_PSI:
    case PARAM_MOTOR_8_PSI:
    case PARAM_MOTOR_9_PSI:
      init_mixing();
      break;
    case PARAM_MOTOR_PWM_SEND_RATE:
    case PARAM_RC_TYPE:
      init_PWM();
      break;
    default:
      // do nothing
      break;
  }
}

void Mixer::init_mixing()
{
  // Update parameters
  update_parameters(); 

  // clear the invalid mixer error
  RF_.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);

  uint8_t mixer_choice = RF_.params_.get_param_int(PARAM_MIXER);

  if (mixer_choice >= NUM_MIXERS) {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR, "Invalid Mixer Choice");

    // set the invalid mixer flag
    RF_.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
    mixer_to_use_ = nullptr;
  } else {
    mixer_to_use_ = array_of_mixers_[mixer_choice];

    if (mixer_to_use_ == &custom_mixing) {
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO, "Calculating custom mixer values...");
      calculate_mixer_values();
      mixer_to_use_ = &correct_mixer_;
    }
  }

  init_PWM();

  for (int8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    raw_outputs_[i] = 0.0f;
    outputs_[i] = 0.0f;
  }
}

void Mixer::update_parameters() {
  R_ = RF_.params_.get_param_float(PARAM_MOTOR_RESISTANCE);
  rho_ = RF_.params_.get_param_float(PARAM_AIR_DENSITY);
  K_V_ = RF_.params_.get_param_float(PARAM_MOTOR_KV);
  K_Q_ = K_V_;
  i_0_ = RF_.params_.get_param_float(PARAM_NO_LOAD_CURRENT);
  D_ = RF_.params_.get_param_float(PARAM_PROP_DIAMETER);
  C_T_ = RF_.params_.get_param_float(PARAM_PROP_CT);
  C_Q_ = RF_.params_.get_param_float(PARAM_PROP_CQ);
  num_motors_ = RF_.params_.get_param_int(PARAM_NUM_MOTORS);
  V_max_ = RF_.params_.get_param_float(PARAM_VOLT_MAX);

  l_[0] = RF_.params_.get_param_float(PARAM_MOTOR_0_POS);
  l_[1] = RF_.params_.get_param_float(PARAM_MOTOR_1_POS);
  l_[2] = RF_.params_.get_param_float(PARAM_MOTOR_2_POS);
  l_[3] = RF_.params_.get_param_float(PARAM_MOTOR_3_POS);
  l_[4] = RF_.params_.get_param_float(PARAM_MOTOR_4_POS);
  l_[5] = RF_.params_.get_param_float(PARAM_MOTOR_5_POS);
  l_[6] = RF_.params_.get_param_float(PARAM_MOTOR_6_POS);
  l_[7] = RF_.params_.get_param_float(PARAM_MOTOR_7_POS);
  l_[8] = RF_.params_.get_param_float(PARAM_MOTOR_8_POS);
  l_[9] = RF_.params_.get_param_float(PARAM_MOTOR_9_POS);

  psi_[0] = RF_.params_.get_param_float(PARAM_MOTOR_0_PSI);
  psi_[1] = RF_.params_.get_param_float(PARAM_MOTOR_1_PSI);
  psi_[2] = RF_.params_.get_param_float(PARAM_MOTOR_2_PSI);
  psi_[3] = RF_.params_.get_param_float(PARAM_MOTOR_3_PSI);
  psi_[4] = RF_.params_.get_param_float(PARAM_MOTOR_4_PSI);
  psi_[5] = RF_.params_.get_param_float(PARAM_MOTOR_5_PSI);
  psi_[6] = RF_.params_.get_param_float(PARAM_MOTOR_6_PSI);
  psi_[7] = RF_.params_.get_param_float(PARAM_MOTOR_7_PSI);
  psi_[8] = RF_.params_.get_param_float(PARAM_MOTOR_8_PSI);
  psi_[9] = RF_.params_.get_param_float(PARAM_MOTOR_9_PSI);
}

void Mixer::calculate_mixer_values()
{
  // Create the mixing matrix as defined in Ch. 14
  Eigen::MatrixXf mixer_matrix(4, num_motors_);
  for (int i = 0; i < num_motors_; ++i) {
    // Determine the direction of the yaw moment generated by the motor i 
    // TODO: This assumes that there are an even number of motors and that they are placed so that every other motor spins the same direction
    int yaw_dir = (i % 2 == 0) ? 1 : -1;

    mixer_matrix(0, i) = rho_ * pow(D_, 4.0) / (4 * pow(M_PI, 2.0)) * C_T_;
    mixer_matrix(1, i) = rho_ * pow(D_, 4.0) / (4 * pow(M_PI, 2.0)) * C_T_ * l_[i] * (-sin(psi_[i]));
    mixer_matrix(2, i) = rho_ * pow(D_, 4.0) / (4 * pow(M_PI, 2.0)) * C_T_ * l_[i] * cos(psi_[i]);
    mixer_matrix(3, i) = rho_ * pow(D_, 5.0) / (4 * pow(M_PI, 2.0)) * yaw_dir * C_Q_;
  }

  // Take the pseudoinverse of the matrix
  Eigen::MatrixXf M_pinv = mixer_matrix.completeOrthogonalDecomposition().pseudoInverse();
  
  // Fill in the mixing matrix from the inverted matrix above 
  for (int i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (i < num_motors_) {
      correct_mixer_.default_pwm_rate[i] = 490;
      correct_mixer_.output_type[i] = M;
      correct_mixer_.F[i] = M_pinv(i, 0);
      correct_mixer_.x[i] = M_pinv(i, 1);
      correct_mixer_.y[i] = M_pinv(i, 2);
      correct_mixer_.z[i] = M_pinv(i, 3);
    }
    else {
      // Set the rest of the ouput types to NONE with a default PWM rate of 50
      correct_mixer_.default_pwm_rate[i] = 50;
      correct_mixer_.output_type[i] = NONE;
      correct_mixer_.F[i] = 0.0;
      correct_mixer_.x[i] = 0.0;
      correct_mixer_.y[i] = 0.0;
      correct_mixer_.z[i] = 0.0;
    }
  }
}

void Mixer::init_PWM()
{
  if (mixer_to_use_ != nullptr) {
    RF_.board_.pwm_init_multi(mixer_to_use_->default_pwm_rate, NUM_MIXER_OUTPUTS);
  } else {
    RF_.board_.pwm_init_multi(passthrough_mixing.default_pwm_rate, NUM_MIXER_OUTPUTS);
  }
}

void Mixer::write_motor(uint8_t index, float value)
{
  if (RF_.state_manager_.state().armed) {
    if (value > 1.0) {
      value = 1.0;
    } else if (value < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)
               && RF_.params_.get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED)) {
      value = RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE);
    } else if (value < 0.0) {
      value = 0.0;
    }
  } else {
    value = 0.0;
  }
  raw_outputs_[index] = value;
  RF_.board_.pwm_write(index, raw_outputs_[index]);
}

void Mixer::write_servo(uint8_t index, float value)
{
  if (value > 1.0) {
    value = 1.0;
  } else if (value < -1.0) {
    value = -1.0;
  }
  raw_outputs_[index] = value;
  RF_.board_.pwm_write(index, raw_outputs_[index] * 0.5 + 0.5);
}

void Mixer::set_new_aux_command(aux_command_t new_aux_command)
{
  for (uint8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    aux_command_.channel[i].type = new_aux_command.channel[i].type;
    aux_command_.channel[i].value = new_aux_command.channel[i].value;
  }
}

void Mixer::mix_output()
{
  Controller::Output commands = RF_.controller_.output();
  float max_output = 1.0f;

  // Reverse fixed-wing channels just before mixing if we need to
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)) {
    commands.x *= RF_.params_.get_param_int(PARAM_AILERON_REVERSE) ? -1 : 1;
    commands.y *= RF_.params_.get_param_int(PARAM_ELEVATOR_REVERSE) ? -1 : 1;
    commands.z *= RF_.params_.get_param_int(PARAM_RUDDER_REVERSE) ? -1 : 1;
  } else if (commands.F < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)) {
    // TODO: Fix this since input to the mixer is thrust, not throttle
    // For multirotors, disregard yaw commands if throttle is low to prevent motor spin-up while
    // arming/disarming
    commands.z = 0.0;
  }

  if (mixer_to_use_ == nullptr) { return; }

  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (mixer_to_use_->output_type[i] != NONE) {
      // Matrix multiply to mix outputs
      double omega_squared = (commands.F * mixer_to_use_->F[i] + commands.x * mixer_to_use_->x[i]
                             + commands.y * mixer_to_use_->y[i] + commands.z * mixer_to_use_->z[i]);
      
      // Ensure that omega_squared is non-negative
      if (omega_squared < 0.0) { omega_squared = 0.0; }
      
      // Ch. 4, setting equation for torque produced by a propeller equal to Eq. 4.19
      // Note that we assume constant airspeed and propeller speed, leading to a constant advance ratio, torque, and thrust constants.
      double V_in = rho_ * pow(D_, 5.0) / (4.0 * pow(M_PI, 2.0)) * omega_squared * C_Q_ * R_ / K_Q_
                    + R_ * i_0_
                    + K_V_ * sqrt(omega_squared);

      // Convert desired V_in setting to a throttle setting
      outputs_[i] = V_in / V_max_;
      
      // Save off the largest control output if it is greater than 1.0 for future scaling
      if (outputs_[i] > max_output) { max_output = outputs_[i]; }
    }
  }

  // saturate outputs to maintain controllability even during aggressive maneuvers
  // TODO: Fix how this saturates to maintain controlability
  float scale_factor = 1.0;
  if (max_output > 1.0) { scale_factor = 1.0 / max_output; }

  // Perform Motor Output Scaling
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    // scale all motor outputs by scale factor (this is usually 1.0, unless we saturated)
    if (mixer_to_use_->output_type[i] == M) { outputs_[i] *= scale_factor; }
  }

  // Insert AUX Commands, and assemble combined_output_types array (Does not override mixer values)

  // For the first NUM_MIXER_OUTPUTS channels, only write aux_command to channels the mixer is not
  // using
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if (mixer_to_use_->output_type[i] == NONE) {
      outputs_[i] = aux_command_.channel[i].value;
      combined_output_type_[i] = aux_command_.channel[i].type;
    } else {
      combined_output_type_[i] = mixer_to_use_->output_type[i];
    }
  }

  // The other channels are never used by the mixer
  for (uint8_t i = NUM_MIXER_OUTPUTS; i < NUM_TOTAL_OUTPUTS; i++) {
    outputs_[i] = aux_command_.channel[i].value;
    combined_output_type_[i] = aux_command_.channel[i].type;
  }

  // Write to outputs
  for (uint8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    float value = outputs_[i];
    if (combined_output_type_[i] == S) {
      if (value > 1.0) {
        value = 1.0;
      } else if (value < -1.0) {
        value = -1.0;
      }
      raw_outputs_[i] = value * 0.5 + 0.5;
    } else if (combined_output_type_[i] == M) {
      if (RF_.state_manager_.state().armed) {
        if (value > 1.0) {
          value = 1.0;
        } else if (value < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)
                   && RF_.params_.get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED)) {
          value = RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE);
        } else if (value < 0.0) {
          value = 0.0;
        }
      } else {
        value = 0.0;
      }
      raw_outputs_[i] = value;
    }
  }
  RF_.board_.pwm_write_multi(raw_outputs_, NUM_TOTAL_OUTPUTS);
}

} // namespace rosflight_firmware
