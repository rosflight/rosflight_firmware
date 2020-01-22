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

#include <stdbool.h>
#include <stdlib.h>

#include "command_manager.h"
#include "rosflight.h"

namespace rosflight_firmware
{

typedef enum
{
  ATT_MODE_RATE,
  ATT_MODE_ANGLE
} att_mode_t;

typedef struct
{
  RC::Stick rc_channel;
  uint32_t last_override_time;
} rc_stick_override_t;

rc_stick_override_t rc_stick_override[] =
{
  { RC::STICK_X, 0 },
  { RC::STICK_Y, 0 },
  { RC::STICK_Z, 0 }
};

typedef struct
{
  control_channel_t *rc;
  control_channel_t *onboard;
  control_channel_t *combined;
} mux_t;

CommandManager::CommandManager(ROSflight &_rf) :
  RF_(_rf),
  failsafe_command_(multirotor_failsafe_command_)
{}

void CommandManager::init()
{
  init_failsafe();
}

void CommandManager::param_change_callback(uint16_t param_id)
{
  switch (param_id)
  {
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
  multirotor_failsafe_command_.F.value = RF_.params_.get_param_float(PARAM_FAILSAFE_THROTTLE);

  if (RF_.params_.get_param_int(PARAM_FIXED_WING))
    failsafe_command_ = fixedwing_failsafe_command_;
  else
    failsafe_command_ = multirotor_failsafe_command_;
}

void CommandManager::interpret_rc(void)
{
  // get initial, unscaled RC values
  rc_command_.x.value = RF_.rc_.stick(RC::STICK_X);
  rc_command_.y.value = RF_.rc_.stick(RC::STICK_Y);
  rc_command_.z.value = RF_.rc_.stick(RC::STICK_Z);
  rc_command_.F.value = RF_.rc_.stick(RC::STICK_F);

  // determine control mode for each channel and scale command values accordingly
  if (RF_.params_.get_param_int(PARAM_FIXED_WING))
  {
    rc_command_.x.type = PASSTHROUGH;
    rc_command_.y.type = PASSTHROUGH;
    rc_command_.z.type = PASSTHROUGH;
    rc_command_.F.type = THROTTLE;
  }
  else
  {
    // roll and pitch
    control_type_t roll_pitch_type;
    if (RF_.rc_.switch_mapped(RC::SWITCH_ATT_TYPE))
    {
      roll_pitch_type = RF_.rc_.switch_on(RC::SWITCH_ATT_TYPE) ? ANGLE : RATE;
    }
    else
    {
      roll_pitch_type = (RF_.params_.get_param_int(PARAM_RC_ATTITUDE_MODE) == ATT_MODE_RATE) ? RATE: ANGLE;
    }

    rc_command_.x.type = roll_pitch_type;
    rc_command_.y.type = roll_pitch_type;

    // Scale command to appropriate units
    switch (roll_pitch_type)
    {
    case RATE:
      rc_command_.x.value *= RF_.params_.get_param_float(PARAM_RC_MAX_ROLLRATE);
      rc_command_.y.value *= RF_.params_.get_param_float(PARAM_RC_MAX_PITCHRATE);
      break;
    case ANGLE:
      rc_command_.x.value *= RF_.params_.get_param_float(PARAM_RC_MAX_ROLL);
      rc_command_.y.value *= RF_.params_.get_param_float(PARAM_RC_MAX_PITCH);
    default:
      break;
    }

    // yaw
    rc_command_.z.type = RATE;
    rc_command_.z.value *= RF_.params_.get_param_float(PARAM_RC_MAX_YAWRATE);

    // throttle
    rc_command_.F.type = THROTTLE;
  }
}

bool CommandManager::stick_deviated(MuxChannel channel)
{
  uint32_t now = RF_.board_.clock_millis();

  // if we are still in the lag time, return true
  if (now  < channel_override_[channel].last_override_time + RF_.params_.get_param_int(PARAM_OVERRIDE_LAG_TIME))
  {
    return true;
  }
  else
  {
    if (fabsf(RF_.rc_.stick(channel_override_[channel].rc_channel))
        > RF_.params_.get_param_float(PARAM_RC_OVERRIDE_DEVIATION))
    {
      channel_override_[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}
uint16_t CommandManager::determine_override_status()
{
  uint16_t rc_override{OVERRIDE_NO_OVERRIDE};
  if(RF_.rc_.switch_mapped(RC::SWITCH_ATT_OVERRIDE) && RF_.rc_.switch_on(RC::SWITCH_ATT_OVERRIDE))
    rc_override |= OVERRIDE_ATT_SWITCH;
  if(RF_.rc_.switch_mapped(RC::SWITCH_THROTTLE_OVERRIDE) && RF_.rc_.switch_on(RC::SWITCH_THROTTLE_OVERRIDE))
    rc_override |= OVERRIDE_THR_SWITCH;
  for(uint8_t channel{0}; channel < MUX_F; channel++)
  {
    if(stick_deviated(static_cast<MuxChannel>(channel)))
      rc_override |= channel_override_[channel].stick_override_reason;
    if(!(muxes[channel].onboard->active))
      rc_override |= channel_override_[channel].offboard_inactive_override_reason;
  }
  if(!(muxes[MUX_F].onboard->active)) // The throttle has unique override behavior
    rc_override |= OVERRIDE_OFFBOARD_T_INACTIVE;
  if(RF_.params_.get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
    if(muxes[MUX_F].rc->value < muxes[MUX_F].onboard->value)
      rc_override |= OVERRIDE_T;
  return rc_override;
}

void CommandManager::do_muxing(uint16_t rc_override)
{
  for(uint8_t channel{0}; channel <=MUX_F; channel++)
    do_channel_muxing(static_cast<MuxChannel>(channel), rc_override);
}
void CommandManager::do_channel_muxing(MuxChannel channel, uint16_t rc_override)
{
  bool override_this_channel = (rc_override & channel_override_[channel].override_mask);
  // set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[channel].combined = override_this_channel ? *muxes[channel].rc : *muxes[channel].onboard;
}

uint16_t CommandManager::get_rc_override()
{
  return rc_override_;
}

bool CommandManager::offboard_control_active()
{
  for (int i = 0; i < 4; i++)
  {
    if (muxes[i].onboard->active)
      return true;
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
  uint16_t last_rc_override = rc_override_;

  // Check for and apply failsafe command
  if (RF_.state_manager_.state().failsafe)
  {
    combined_command_ = failsafe_command_;
  }
  else if (RF_.rc_.new_command())
  {
    // Read RC
    interpret_rc();

    // Check for offboard control timeout (100 ms)
    if (RF_.board_.clock_millis() > offboard_command_.stamp_ms + RF_.params_.get_param_int(PARAM_OFFBOARD_TIMEOUT))
    {
      // If it has been longer than 100 ms, then disable the offboard control
      offboard_command_.F.active = false;
      offboard_command_.x.active = false;
      offboard_command_.y.active = false;
      offboard_command_.z.active = false;
    }

    // Perform muxing
    rc_override_ = determine_override_status();
    do_muxing(rc_override_);

    // Light to indicate override
    if (rc_override_)
    {
      RF_.board_.led0_on();
    }
    else
    {
      RF_.board_.led0_off();
    }
  }

  // There was a change in rc_override state
  if (last_rc_override != rc_override_)
  {
    RF_.comm_manager_.update_status();
  }
  return true;
}


}
