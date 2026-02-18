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

namespace rosflight_firmware
{
Mixer::Mixer(ROSflight & _rf)
    : RF_(_rf)
{
  for (int8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    aux_command_.channel[i].type = AUX;
    aux_command_.channel[i].value = 0.0f;
  }
}

void Mixer::init()
{
  load_primary_mixer_values();
  load_secondary_mixer_values();

  init_mixing();
}

void Mixer::param_change_callback(uint16_t param_id)
{
  // Follows parameter ordered in mixer.h

  if ((param_id >=PARAM_PRIMARY_MIXER_OUTPUT_0 )&&(param_id <=PARAM_PRIMARY_MIXER_OUTPUT_9 )) {
    primary_mixer_.output_type[param_id-PARAM_PRIMARY_MIXER_OUTPUT_0] = (output_type_t) RF_.params_.get_param_int(param_id);
    // Note: secondary mixer output types are not used, but should match the primary mixer in case code elsewhere changes.
    secondary_mixer_.output_type[param_id-PARAM_PRIMARY_MIXER_OUTPUT_0] = (output_type_t) RF_.params_.get_param_int(param_id);

  } else if ((param_id >=PARAM_PRIMARY_MIXER_PWM_RATE_0 )&&(param_id <=PARAM_PRIMARY_MIXER_PWM_RATE_9 )) {
    primary_mixer_.default_pwm_rate[param_id-PARAM_PRIMARY_MIXER_PWM_RATE_0] = RF_.params_.get_param_float(param_id);
    // Note: secondary mixer pwm rates are not used, but should match the primary mixer in case code elsewhere changes.
    secondary_mixer_.default_pwm_rate[param_id-PARAM_PRIMARY_MIXER_PWM_RATE_0] = RF_.params_.get_param_float(param_id);

  } else if ((param_id >=PARAM_PRIMARY_MIXER_0_0 )&&(param_id <=PARAM_PRIMARY_MIXER_9_9 )) {

    uint16_t param_id_offset =  param_id - PARAM_PRIMARY_MIXER_0_0;
    float param_value = RF_.params_.get_param_float(param_id);
    uint16_t param_id_row = param_id_offset % NUM_MIXER_OUTPUTS;
    uint16_t param_id_col = param_id_offset / NUM_MIXER_OUTPUTS;
    primary_mixer_.u[param_id_row][param_id_col] = param_value;

    // Special Case for when secondary mixer is mirroring primary mixer.
    mixer_type_t mixer_choice = static_cast<mixer_type_t>(RF_.params_.get_param_int(PARAM_SECONDARY_MIXER));
    if (mixer_choice >= NUM_MIXERS) {
      secondary_mixer_.u[param_id_row][param_id_col] = param_value;

      // Write the value to params -- otherwise, the secondary mixer gets out of sync
      uint16_t secondary_mixer_param_id = PARAM_SECONDARY_MIXER_0_0 + param_id_col*NUM_MIXER_OUTPUTS + param_id_row;
      RF_.params_.set_param_float(secondary_mixer_param_id, param_value);
    }

  } else if ((param_id >=PARAM_SECONDARY_MIXER_0_0 )&&(param_id <=PARAM_SECONDARY_MIXER_9_9 )) {

    uint16_t param_id_offset =  param_id - PARAM_SECONDARY_MIXER_0_0;
    uint16_t param_id_row = param_id_offset % NUM_MIXER_OUTPUTS;
    uint16_t param_id_col = param_id_offset / NUM_MIXER_OUTPUTS;
    secondary_mixer_.u[param_id_row][param_id_col] = RF_.params_.get_param_float(param_id);

  } else switch (param_id) {
    case PARAM_PRIMARY_MIXER:
    case PARAM_SECONDARY_MIXER:
    case PARAM_USE_MOTOR_PARAMETERS:
    case PARAM_RC_F_AXIS:
      init_mixing();
      break;
    case PARAM_MOTOR_RESISTANCE: R_ = RF_.params_.get_param_float(PARAM_MOTOR_RESISTANCE); break;
    case PARAM_MOTOR_KV: K_V_ = RF_.params_.get_param_float(PARAM_MOTOR_KV); K_Q_ = K_V_; break;
    case PARAM_MOTOR_NO_LOAD_CURRENT: i_0_ = RF_.params_.get_param_float(PARAM_MOTOR_NO_LOAD_CURRENT); break;
    case PARAM_PROP_DIAMETER: D_ = RF_.params_.get_param_float(PARAM_PROP_DIAMETER); break;
    case PARAM_PROP_CT: C_T_ = RF_.params_.get_param_float(PARAM_PROP_CT); break;
    case PARAM_PROP_CQ: C_Q_ = RF_.params_.get_param_float(PARAM_PROP_CQ); break;
    case PARAM_NUM_MOTORS: num_motors_ = RF_.params_.get_param_int(PARAM_NUM_MOTORS); break;
      init_PWM();
      break;
    default:
      // do nothing
      break;
  }
}


void Mixer::init_mixing()
{
  // clear the invalid mixer error
  RF_.state_manager_.clear_error(StateManager::ERROR_INVALID_MIXER);

  // Set up the primary mixer, used by the RC safety pilot
  mixer_type_t mixer_choice = static_cast<mixer_type_t>(RF_.params_.get_param_int(PARAM_PRIMARY_MIXER));

  if (mixer_choice >= NUM_MIXERS) {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR, "Invalid mixer choice for primary mixer");

    // set the invalid mixer flag
    RF_.state_manager_.set_error(StateManager::ERROR_INVALID_MIXER);
    primary_mixer_is_selected_ = false;
  } else {
    if (mixer_choice == CUSTOM) {
      // Load the custom mixer for the primary mixer based off the saved parameters.
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                            "Loading saved custom values to primary mixer...");

      load_primary_mixer_values();

      // Update the motor parameters that will be used
      update_parameters();
    } else if (mixer_choice != FIXEDWING &&
               mixer_choice != INVERTED_VTAIL) {
      // Invert the selected "canned" matrix
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,"Inverting selected mixing matrix...");
      primary_mixer_ = invert_mixer(array_of_mixers_[mixer_choice]);

      // Save the primary mixer values to the params
      save_primary_mixer_params();

      // If using a canned mixer but the USE_MOTOR_PARAM is set to 1 (true), raise a warning.
      // Motor parameters (thus motor speed/voltage calculations) should not be used with the canned
      // mixers, since the output will be vanishingly small. Check online documentation for more 
      // information.
      if (!(RF_.params_.get_param_int(PARAM_USE_MOTOR_PARAMETERS) == 0)) {
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                              ("USE_MOTOR_PARAM=1, but PRIMARY_MIXER=" + std::to_string(static_cast<unsigned int>(mixer_choice)) + ", which").c_str());
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                              "may cause issues (check docs). Is this correct?");
      }

    } else {
      // Don't invert the fixedwing mixers
      primary_mixer_ = *array_of_mixers_[mixer_choice];

      // Save the primary mixer values to the params
      save_primary_mixer_params();

      // For the fixedwing canned mixers, the RC_F_AXIS parameter should be set to 0 (X-AXIS).
      // Otherwise, the aircraft will arm and appear to be ok, but will zero out any RC throttle
      // commands. Raise a warning if this condition is detected
      int rc_f_axis = RF_.params_.get_param_int(PARAM_RC_F_AXIS);
      if (!(rc_f_axis == 0)) {
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                              ("PRIMARY_MIXER=" + std::to_string(static_cast<unsigned int>(mixer_choice))
                              + " but RC_F_AXIS="
                              + std::to_string(static_cast<unsigned int>(rc_f_axis))
                              + ", which"
                              ).c_str());
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                              "will cause issues (check docs). Is this correct?");
      }
    }

    // Load the primary mixer header to the mixer_to_use_ header. Note that both the primary and 
    // secondary mixers will use the header for the primary mixer
    mixer_to_use_.output_type = &primary_mixer_.output_type;
    mixer_to_use_.default_pwm_rate = &primary_mixer_.default_pwm_rate;

    // Load the primary mixing values into the mixer_to_use_ by default
    for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
      mixer_to_use_.u[i] = primary_mixer_.u[i];
    }

    primary_mixer_is_selected_ = true;
  }

  // Compute or load the secondary mixer, used by the offboard control
  mixer_choice = static_cast<mixer_type_t>(RF_.params_.get_param_int(PARAM_SECONDARY_MIXER));

  if (mixer_choice >= NUM_MIXERS) {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
        "Invalid mixer selected for secondary mixer!");
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
        "Secondary mixer defaulting to primary!");
    
    secondary_mixer_ = primary_mixer_;
    save_secondary_mixer_params();
  } else if (mixer_choice == CUSTOM) {
    // Load the custom mixer for the secondary mixer based off the saved parameters.
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                          "Loading saved custom values to secondary mixer");

    load_secondary_mixer_values();
  } else if (mixer_choice != FIXEDWING ||
             mixer_choice != INVERTED_VTAIL) {
    // Invert the selected "canned" matrix
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_INFO,
                          "Inverting selected mixing matrix...");

    // Invert the selected mixer and copy to secondary mixer
    secondary_mixer_ = invert_mixer(array_of_mixers_[mixer_choice]);
    save_secondary_mixer_params();
  } else {
    // Don't invert the fixedwing mixers
    secondary_mixer_ = *array_of_mixers_[mixer_choice];
    save_secondary_mixer_params();
  }

  init_PWM();

  for (int8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    raw_outputs_[i] = 0.0f;
    outputs_[i] = 0.0f;
  }
}

void Mixer::update_parameters()
{
  R_ = RF_.params_.get_param_float(PARAM_MOTOR_RESISTANCE);
  K_V_ = RF_.params_.get_param_float(PARAM_MOTOR_KV);
  K_Q_ = K_V_;
  i_0_ = RF_.params_.get_param_float(PARAM_MOTOR_NO_LOAD_CURRENT);
  D_ = RF_.params_.get_param_float(PARAM_PROP_DIAMETER);
  C_T_ = RF_.params_.get_param_float(PARAM_PROP_CT);
  C_Q_ = RF_.params_.get_param_float(PARAM_PROP_CQ);
  num_motors_ = RF_.params_.get_param_int(PARAM_NUM_MOTORS);
}

Mixer::mixer_t Mixer::invert_mixer(const mixer_t* mixer_to_invert)
{
  Eigen::Matrix<float, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS> mixer_matrix;
  mixer_matrix.setZero();

  // Convert the mixer_t to an Eigen matrix
  for (int i=0; i<NUM_MIXER_OUTPUTS; i++) {
    for (int j=0; j<NUM_MIXER_OUTPUTS; j++) {
      mixer_matrix(i, j) = mixer_to_invert->u[i][j];
    }
  }

  // Calculate the pseudoinverse of the mixing matrix using the SVD
  Eigen::JacobiSVD<Eigen::Matrix<float, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS>> svd(
    mixer_matrix,
    Eigen::FullPivHouseholderQRPreconditioner | Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<float, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS> Sig;
  Sig.setZero();

  // Avoid dividing by zero in the Sigma matrix
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    if (svd.singularValues()[i] != 0.0) { Sig(i, i) = 1.0 / svd.singularValues()[i]; }
  }

  // Pseudoinverse of the mixing matrix
  Eigen::Matrix<float, NUM_MIXER_OUTPUTS, NUM_MIXER_OUTPUTS> mixer_matrix_pinv =
    svd.matrixV() * Sig * svd.matrixU().transpose();

  mixer_t inverted_mixer;
  // Fill in the mixing matrix from the inverted matrix above
  for (int i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    inverted_mixer.output_type[i] = mixer_to_invert->output_type[i];
    inverted_mixer.default_pwm_rate[i] = mixer_to_invert->default_pwm_rate[i];
    for (int j = 0; j < NUM_MIXER_OUTPUTS; j++) {
      inverted_mixer.u[j][i] = mixer_matrix_pinv(i, j);
    }
  }
  return inverted_mixer;
}

void Mixer::save_primary_mixer_params()
{
  // Save the mixer header values
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    // This assumes the parameters are stored in order in the param enum
    int output_param_index = (int) PARAM_PRIMARY_MIXER_OUTPUT_0 + i;
    int pwm_rate_param_index = (int) PARAM_PRIMARY_MIXER_PWM_RATE_0 + i;
    RF_.params_.set_param_int(output_param_index, primary_mixer_.output_type[i]);
    RF_.params_.set_param_float(pwm_rate_param_index, primary_mixer_.default_pwm_rate[i]);
  }

  // Save the mixer values to the firmware parameters
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    // This assumes the parameters are stored in order in the param enum
    int param_index = (int) PARAM_PRIMARY_MIXER_0_0 + NUM_MIXER_OUTPUTS * i;
    for (int j = 0; j < NUM_MIXER_OUTPUTS; j++) {
      RF_.params_.set_param_float(param_index++, primary_mixer_.u[j][i]);
    }
  }
}

void Mixer::save_secondary_mixer_params()
{
  // Save the mixer values to the firmware parameters
  // The secondary mixer does not have header values (they are the same as the primary mixer)
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    // This assumes the parameters are stored in order in the param enum
    int param_index = (int) PARAM_SECONDARY_MIXER_0_0 + NUM_MIXER_OUTPUTS * i;
    for (int j = 0; j < NUM_MIXER_OUTPUTS; j++) {
      RF_.params_.set_param_float(param_index++, secondary_mixer_.u[j][i]);
    }
  }
}

void Mixer::load_primary_mixer_values()
{
  // Load the mixer header values
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    primary_mixer_.output_type[i] = (output_type_t) RF_.params_.get_param_int(PARAM_PRIMARY_MIXER_OUTPUT_0 + i);
    primary_mixer_.default_pwm_rate[i] = RF_.params_.get_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_0 + i);
  }

  // Load the mixer values from the firmware parameters
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    // This assumes the parameters are stored in order in the param enum
    int param_index = (int) PARAM_PRIMARY_MIXER_0_0 + NUM_MIXER_OUTPUTS * i;
    for (int j = 0; j < NUM_MIXER_OUTPUTS; j++) {
      primary_mixer_.u[j][i] = RF_.params_.get_param_float(param_index++);
    }
  }
}

void Mixer::load_secondary_mixer_values()
{
  // Load the mixer values from the firmware parameters
  // The header values will be the same as the primary mixer
  for (int i=0; i<NUM_MIXER_OUTPUTS; ++i) {
    // This assumes the parameters are stored in order in the param enum
    int param_index = (int) PARAM_SECONDARY_MIXER_0_0 + NUM_MIXER_OUTPUTS * i;
    for (int j = 0; j < NUM_MIXER_OUTPUTS; j++) {
      secondary_mixer_.u[j][i] = RF_.params_.get_param_float(param_index++);
    }
  }
}

void Mixer::init_PWM()
{
  if (primary_mixer_is_selected_) {
    RF_.board_.pwm_init(primary_mixer_.default_pwm_rate, NUM_MIXER_OUTPUTS);
  } else {
    RF_.board_.pwm_init(esc_calibration_mixing.default_pwm_rate, NUM_MIXER_OUTPUTS);
  }
}

void Mixer::set_new_aux_command(aux_command_t new_aux_command)
{
  for (uint8_t i = 0; i < NUM_TOTAL_OUTPUTS; i++) {
    aux_command_.channel[i].type = new_aux_command.channel[i].type;
    aux_command_.channel[i].value = new_aux_command.channel[i].value;
  }
}

float Mixer::mix_multirotor_without_motor_parameters(Controller::Output commands)
{
  // Mix the inputs
  float max_output{1.0};

  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if ((*mixer_to_use_.output_type)[i] != AUX) {
      outputs_[i] = 0.0;
      // Matrix multiply to mix outputs
      for (uint8_t j = 0; j < NUM_MIXER_OUTPUTS; j++) {
        outputs_[i] += commands.u[j] * mixer_to_use_.u[j][i];
      }

      // Save off the largest control output (for motors) if it is greater than 1.0 for future scaling
      if ((*mixer_to_use_.output_type)[i] == M) {
        if (abs(outputs_[i]) > max_output) { max_output = abs(outputs_[i]); }
      }
    }
  }

  return max_output;
}

float Mixer::mix_multirotor_with_motor_parameters(Controller::Output commands)
{
  // Mix the inputs
  float max_output{1.0};
  float rho = RF_.sensors_.rho();

  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if ((*mixer_to_use_.output_type)[i] == M) {
      float omega_squared = 0.0;
      // Matrix multiply to mix outputs for Motor type
      for (uint8_t j = 0; j < NUM_MIXER_OUTPUTS; j++) {
        omega_squared += commands.u[j] * mixer_to_use_.u[j][i];
      }

      // Ensure that omega_squared is non-negative
      if (omega_squared < 0.0) { omega_squared = 0.0; }

      if (K_Q_ < 0.0000001) {
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                              "Divide by K_V, which is zero! Skipping");
        outputs_[i] = 0.0;
        continue;
      }

      // Ch. 4, setting equation for torque produced by a propeller equal to Eq. 4.19
      // Note that we assume constant advance ratio, leading to constant torque and thrust constants.
      float V_in = rho * pow(D_, 5.0) / (4.0 * pow(M_PI, 2.0)) * omega_squared * C_Q_ * R_ / K_Q_
        + R_ * i_0_ + K_V_ * sqrt(omega_squared);

      // Convert desired V_in setting to a throttle setting
      BatteryStruct * batt = RF_.sensors_.get_battery();
      if (batt->voltage < 0.0001) {
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                              "Divide by battery voltage, which is near zero!");
        RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_ERROR,
                              "Is battery multiplier set correctly?");
        outputs_[i] = 0.0;
        continue;
      }

      outputs_[i] = V_in / batt->voltage;

      // Save off the largest control output (for motors) if it is greater than 1.0 for future scaling
      if (abs(outputs_[i]) > max_output) { max_output = abs(outputs_[i]); }

    } else if ((*mixer_to_use_.output_type)[i] == S) {
      outputs_[i] = 0.0;
      // Matrix multiply to mix outputs for Servo type
      for (uint8_t j = 0; j < NUM_MIXER_OUTPUTS; j++) {
        outputs_[i] += commands.u[j] * mixer_to_use_.u[j][i];
      }
    }
  }

  return max_output;
}

void Mixer::mix_multirotor()
{
  Controller::Output commands = RF_.controller_.output();

  // Check the throttle command based on the axis corresponding to the RC F channel
  float throttle_command{0.0};
  switch (static_cast<rc_f_axis_t>(RF_.params_.get_param_int(PARAM_RC_F_AXIS))) {
    case X_AXIS:
      throttle_command = commands.u[0];
      break;
    case Y_AXIS:
      throttle_command = commands.u[1];
      break;
    case Z_AXIS:
      throttle_command = commands.u[2];
      break;
    default:
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
          "Invalid RC F axis. Defaulting to z-axis.");
      throttle_command = commands.u[2]; // Fz
      break;
  }
  
  if (abs(throttle_command) < RF_.params_.get_param_float(PARAM_MOTOR_IDLE_THROTTLE)) {
    // For multirotors, disregard yaw commands if throttle is low to prevent motor spin-up while
    // arming/disarming
    commands.u[5] = 0.0; // Qz
  }

  // Mix the outputs based on if a custom mixer (i.e. with motor parameters) is selected.
  float max_output{1.0};
  if (RF_.params_.get_param_int(PARAM_USE_MOTOR_PARAMETERS)) {
    max_output = mix_multirotor_with_motor_parameters(commands);
  } else {
    max_output = mix_multirotor_without_motor_parameters(commands);
  }
  
  // Check to see if the max_output is large. If it is, something is likely wrong with the mixer configuration, so
  // warn the user. Note that 2 is an arbitrary value, but seems like a good upper limit since the max output should usually
  // be between 0 and 1.
  if (max_output > 2.0) {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING, "Output from mixer is %f! Check mixer", max_output);
  }

  // There is no relative scaling on the above equations. In other words, if the input F command is too
  // high, then it will "drown out" all other desired outputs. Therefore, we saturate motor outputs to 
  // maintain controllability even during aggressive maneuvers.
  float scale_factor{1.0};
  if (max_output > 1.0) { scale_factor = 1.0 / max_output; }

  // Perform Motor Output Scaling
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    // scale all motor outputs by scale factor (this is usually 1.0, unless we saturated)
    if ((*mixer_to_use_.output_type)[i] == M) { outputs_[i] *= scale_factor; }
  }
}

void Mixer::mix_fixedwing()
{
  Controller::Output commands = RF_.controller_.output();
  
  // Reverse fixed-wing channels just before mixing if we need to
  commands.u[3] *= RF_.params_.get_param_int(PARAM_AILERON_REVERSE) ? -1 : 1;
  commands.u[4] *= RF_.params_.get_param_int(PARAM_ELEVATOR_REVERSE) ? -1 : 1;
  commands.u[5] *= RF_.params_.get_param_int(PARAM_RUDDER_REVERSE) ? -1 : 1;

  // Mix the outputs
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if ((*mixer_to_use_.output_type)[i] != AUX) {
      outputs_[i] = 0.0;
      for (uint8_t j = 0; j < NUM_MIXER_OUTPUTS; j++) {
        outputs_[i] += commands.u[j] * mixer_to_use_.u[j][i];
      }
    }
  }
}

void Mixer::select_primary_or_secondary_mixer()
{
  // Check if under RC control. If RC throttle or attitude override is active,
  // adjust the mixer_to_use_ accordingly.
  uint16_t rc_override = RF_.command_manager_.get_rc_override();

  uint16_t rc_attitude_override_active = static_cast<uint16_t>(rc_override & CommandManager::ATTITUDE_OVERRIDDEN);
  if (rc_attitude_override_active != 0) {
    mixer_to_use_.u[3] = primary_mixer_.u[3];
    mixer_to_use_.u[4] = primary_mixer_.u[4];
    mixer_to_use_.u[5] = primary_mixer_.u[5];
  } else {
    mixer_to_use_.u[3] = secondary_mixer_.u[3];
    mixer_to_use_.u[4] = secondary_mixer_.u[4];
    mixer_to_use_.u[5] = secondary_mixer_.u[5];
  }

  uint16_t rc_throttle_override_active = static_cast<uint16_t>(rc_override & CommandManager::T_OVERRIDDEN);
  if (rc_throttle_override_active != 0) {
    mixer_to_use_.u[0] = primary_mixer_.u[0];
    mixer_to_use_.u[1] = primary_mixer_.u[1];
    mixer_to_use_.u[2] = primary_mixer_.u[2];
  } else {
    mixer_to_use_.u[0] = secondary_mixer_.u[0];
    mixer_to_use_.u[1] = secondary_mixer_.u[1];
    mixer_to_use_.u[2] = secondary_mixer_.u[2];
  }
}

void Mixer::mix_output()
{
  if (!primary_mixer_is_selected_) { return; }

  // Select the primary or secondary mixer based on the RC override status
  select_primary_or_secondary_mixer();

  // Mix according to airframe type
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)) {
    mix_fixedwing();
  } else {
    mix_multirotor();
  }

  // Insert AUX Commands, and assemble combined_output_types array (Does not override mixer values)
  // For the first NUM_MIXER_OUTPUTS channels, only write aux_command to channels the mixer is not
  // using
  for (uint8_t i = 0; i < NUM_MIXER_OUTPUTS; i++) {
    if ((*mixer_to_use_.output_type)[i] == AUX) {
      outputs_[i] = aux_command_.channel[i].value;
      combined_output_type_[i] = aux_command_.channel[i].type;
    } else {
      combined_output_type_[i] = (*mixer_to_use_.output_type)[i];
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
}

} // namespace rosflight_firmware
