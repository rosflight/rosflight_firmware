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

#include "command_manager.h"

#include "rosflight.h"

#include <cstdbool>
#include <cstdlib>

namespace rosflight_firmware
{

CommandManager::CommandManager(ROSflight & _rf)
    : RF_(_rf)
    , failsafe_command_(multirotor_failsafe_command_)
{}

void CommandManager::init()
{
  init_failsafe();
  rc_override_ = determine_override_status();
}

void CommandManager::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_FIXED_WING:
    case PARAM_FAILSAFE_THROTTLE:
      init_failsafe();
      break;
    default:
      // do nothing
      break;
  }
}

void CommandManager::init_failsafe()
{
  float failsafe_thr_param = RF_.params_.get_param_float(PARAM_FAILSAFE_THROTTLE);
  bool fixedwing = RF_.params_.get_param_int(PARAM_FIXED_WING);
  // fixed wings always have 0 failsafe throttle
  if (!fixedwing && (failsafe_thr_param < 0. || failsafe_thr_param > 1.0)) {
    RF_.state_manager_.set_error(StateManager::ERROR_INVALID_FAILSAFE);
    failsafe_thr_param = 0.;
  } else {
    RF_.state_manager_.clear_error(StateManager::ERROR_INVALID_FAILSAFE);
  }

  // Make sure the failsafe is set to the axis associated with the RC F command
  switch (static_cast<rc_f_axis_t>(RF_.params_.get_param_int(PARAM_RC_F_AXIS))) {
    case X_AXIS:
      multirotor_failsafe_command_.u[0].value = failsafe_thr_param;
      break;
    case Y_AXIS:
      multirotor_failsafe_command_.u[1].value = failsafe_thr_param;
      break;
    case Z_AXIS:
      multirotor_failsafe_command_.u[2].value = failsafe_thr_param;
      break;
    default:
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
          "Invalid RC F axis. Defaulting to z-axis.");
      multirotor_failsafe_command_.u[2].value = failsafe_thr_param;
      break;
  }

  if (fixedwing) {
    failsafe_command_ = fixedwing_failsafe_command_;
  } else {
    failsafe_command_ = multirotor_failsafe_command_;
  }
}

void CommandManager::interpret_rc(void)
{
  // get initial, unscaled RC values
  rc_command_.u[3].value = RF_.rc_.stick(RC::STICK_X);
  rc_command_.u[4].value = RF_.rc_.stick(RC::STICK_Y);
  rc_command_.u[5].value = RF_.rc_.stick(RC::STICK_Z);

  // Load the RC command based on the axis associated with the RC F command
  rc_command_.u[0].value = 0.0;
  rc_command_.u[1].value = 0.0;
  rc_command_.u[2].value = 0.0;
  switch (static_cast<rc_f_axis_t>(RF_.params_.get_param_int(PARAM_RC_F_AXIS))) {
    case X_AXIS:   // RC F = X axis
      rc_command_.u[0].value = RF_.rc_.stick(RC::STICK_F);
      break;
    case Y_AXIS:   // RC F = Y axis
      rc_command_.u[1].value = RF_.rc_.stick(RC::STICK_F);
      break;
    case Z_AXIS:
      rc_command_.u[2].value = RF_.rc_.stick(RC::STICK_F);
      break;
    default:
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
          "Invalid RC F axis. Defaulting to z-axis.");
      rc_command_.u[2].value = RF_.rc_.stick(RC::STICK_F);
      break;
  }

  // determine control mode for each channel and scale command values accordingly
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)) {
    rc_command_.u[0].type = PASSTHROUGH;
    rc_command_.u[1].type = PASSTHROUGH;
    rc_command_.u[2].type = PASSTHROUGH;
    rc_command_.u[3].type = PASSTHROUGH;
    rc_command_.u[4].type = PASSTHROUGH;
    rc_command_.u[5].type = PASSTHROUGH;
  } else {
    // roll and pitch
    control_type_t roll_pitch_type;
    if (RF_.rc_.switch_mapped(RC::SWITCH_ATT_TYPE)) {
      roll_pitch_type = RF_.rc_.switch_on(RC::SWITCH_ATT_TYPE) ? ANGLE : RATE;
    } else {
      roll_pitch_type =
        (RF_.params_.get_param_int(PARAM_RC_ATTITUDE_MODE) == ATT_MODE_RATE) ? RATE : ANGLE;
    }

    rc_command_.u[3].type = roll_pitch_type;
    rc_command_.u[4].type = roll_pitch_type;

    // Scale command to appropriate units
    switch (roll_pitch_type) {
      case RATE:
        rc_command_.u[3].value *= RF_.params_.get_param_float(PARAM_RC_MAX_ROLLRATE);
        rc_command_.u[4].value *= RF_.params_.get_param_float(PARAM_RC_MAX_PITCHRATE);
        break;
      case ANGLE:
        rc_command_.u[3].value *= RF_.params_.get_param_float(PARAM_RC_MAX_ROLL);
        rc_command_.u[4].value *= RF_.params_.get_param_float(PARAM_RC_MAX_PITCH);
      default:
        break;
    }

    // yaw
    rc_command_.u[5].type = RATE;
    rc_command_.u[5].value *= RF_.params_.get_param_float(PARAM_RC_MAX_YAWRATE);

    // throttle
    rc_command_.u[0].type = THROTTLE;
    rc_command_.u[1].type = THROTTLE;
    rc_command_.u[2].type = THROTTLE;
  }
}

bool CommandManager::stick_deviated(MuxChannel channel)
{
  uint32_t now = RF_.board_.clock_millis();

  // if we are still in the lag time, return true
  if (now < channel_override_[channel].last_override_time
        + RF_.params_.get_param_int(PARAM_OVERRIDE_LAG_TIME)) {
    return true;
  } else {
    if (fabsf(RF_.rc_.stick(channel_override_[channel].rc_channel))
        > RF_.params_.get_param_float(PARAM_RC_OVERRIDE_DEVIATION)) {
      channel_override_[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}

uint16_t CommandManager::determine_override_status()
{
  // Check if the override switch exists and is triggered, or if the sticks have deviated enough to
  // trigger an override
  uint16_t rc_override{0};
  if (RF_.rc_.switch_mapped(RC::SWITCH_ATT_OVERRIDE) && RF_.rc_.switch_on(RC::SWITCH_ATT_OVERRIDE)) {
    rc_override |= OVERRIDE_ATT_SWITCH;
  }
  if (RF_.rc_.switch_mapped(RC::SWITCH_THROTTLE_OVERRIDE) && RF_.rc_.switch_on(RC::SWITCH_THROTTLE_OVERRIDE)) {
    rc_override |= OVERRIDE_THR_SWITCH;
  }

  rc_override |= check_if_attitude_channel_is_overridden_by_rc(MUX_QX);
  rc_override |= check_if_attitude_channel_is_overridden_by_rc(MUX_QY);
  rc_override |= check_if_attitude_channel_is_overridden_by_rc(MUX_QZ);
  rc_override |= check_if_throttle_channel_is_overridden_by_rc();

  return rc_override;
}

uint16_t CommandManager::check_if_attitude_channel_is_overridden_by_rc(MuxChannel channel)
{
  uint16_t rc_overrides{0};
  if (stick_deviated(channel)) {
    rc_overrides |= channel_override_[channel].stick_override_reason;
  }
  if (!(muxes_[channel].onboard->active)) {
    rc_overrides |= channel_override_[channel].offboard_inactive_override_reason;
  }
  return rc_overrides;
}

uint16_t CommandManager::check_if_throttle_channel_is_overridden_by_rc()
{
  MuxChannel selected_channel;
  // Determine which channel to check based on which axis the RC F channel corresponds to
  switch (static_cast<rc_f_axis_t>(RF_.params_.get_param_int(PARAM_RC_F_AXIS))) {
    case X_AXIS:
      selected_channel = MUX_FX;
      break;
    case Y_AXIS:
      selected_channel = MUX_FY;
      break;
    case Z_AXIS:
      selected_channel = MUX_FZ;
      break;
    default:
      RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
          "Invalid RC F axis. Defaulting to z-axis.");
      selected_channel = MUX_FZ;
      break;
  }

  uint16_t rc_overrides{0};
  if (!(muxes_[selected_channel].onboard->active)) {
    rc_overrides |= OVERRIDE_OFFBOARD_T_INACTIVE;
  }
  if (RF_.params_.get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE)) {
    if (muxes_[selected_channel].rc->value < muxes_[selected_channel].onboard->value) {
      rc_overrides |= OVERRIDE_T;
    }
  }
  return rc_overrides;
}

void CommandManager::do_muxing(uint16_t rc_override)
{
  for (uint8_t channel{0}; channel < NUM_MUX_CHANNELS; ++channel) {
    do_channel_muxing(static_cast<MuxChannel>(channel), rc_override);
  }
  // Pass the remaining channels that RC does not override through to the combined command vector.
  for (uint8_t channel{NUM_MUX_CHANNELS}; channel < Mixer::NUM_MIXER_OUTPUTS; ++channel) {
    *muxes_[channel].combined = *muxes_[channel].onboard;
  }
}

void CommandManager::do_channel_muxing(MuxChannel channel, uint16_t rc_override )
{
  bool override_this_channel = (rc_override & channel_override_[channel].override_mask);
  // set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes_[channel].combined = override_this_channel ? *muxes_[channel].rc : *muxes_[channel].onboard;
}

uint16_t CommandManager::get_rc_override() const { return rc_override_; }

bool CommandManager::offboard_control_active()
{
  for (uint8_t channel{0}; channel < NUM_MUX_CHANNELS; ++channel) {
    if (muxes_[channel].onboard->active) { return true; }
  }
  return false;
}

void CommandManager::set_new_offboard_command(control_t new_offboard_command)
{
  new_command_ = true;
  offboard_command_ = new_offboard_command;
}

void CommandManager::set_new_rc_command(control_t new_rc_command)
{
  new_command_ = true;
  rc_command_ = new_rc_command;
}

void CommandManager::override_combined_command_with_rc()
{
  new_command_ = true;
  combined_command_ = rc_command_;
}

bool CommandManager::run()
{
  uint16_t last_rc_override = get_rc_override();

  // Check for and apply failsafe command
  if (RF_.state_manager_.state().failsafe) {
    combined_command_ = failsafe_command_;
  } else if (RF_.rc_.new_command()) {
    // Read RC
    interpret_rc();

    // Check for offboard control timeout (100 ms)
    if (RF_.board_.clock_millis()
        > offboard_command_.stamp_ms + RF_.params_.get_param_int(PARAM_OFFBOARD_TIMEOUT)) {
      // If it has been longer than 100 ms, then disable the offboard control
      for (int i=0; i<Mixer::NUM_MIXER_OUTPUTS; ++i) {
        offboard_command_.u[i].active = false;
      }
    }

    // Perform muxing
    rc_override_ = determine_override_status();
    do_muxing(rc_override_);

    // Light to indicate override
    if (rc_override_) {
      RF_.board_.led0_on();
    } else {
      RF_.board_.led0_off();
    }
  }

  // There was a change in rc_override state
  if (last_rc_override != get_rc_override()) { RF_.comm_manager_.update_status(); }
  return true;
}

} // namespace rosflight_firmware
