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
  RC::rc_stick_t rc_channel;
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

CommandManager::CommandManager(ROSflight& _rf) :
  RF_(_rf)
{}

void CommandManager::init()
{}

void CommandManager::interpret_rc(void)
{
  // get initial, unscaled RC values
  rc_command_.x.value = RF_.rc_.rc_stick(RC::STICK_X);
  rc_command_.y.value = RF_.rc_.rc_stick(RC::STICK_Y);
  rc_command_.z.value = RF_.rc_.rc_stick(RC::STICK_Z);
  rc_command_.F.value = RF_.rc_.rc_stick(RC::STICK_F);

  // determine control mode for each channel and scale command values accordingly
  if (RF_.params_.get_param_int(PARAM_FIXED_WING))
  {
    rc_command_.x.type = PASSTHROUGH;
    rc_command_.y.type = PASSTHROUGH;
    rc_command_.z.type = PASSTHROUGH;
  }
  else
  {
    // roll and pitch
    control_type_t roll_pitch_type;
    if (RF_.rc_.rc_switch_mapped(RC::SWITCH_ATT_TYPE))
    {
      roll_pitch_type = RF_.rc_.rc_switch(RC::SWITCH_ATT_TYPE) ? ANGLE : RATE;
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
    }

    // yaw
    rc_command_.z.type = RATE;
    rc_command_.z.value *= RF_.params_.get_param_float(PARAM_RC_MAX_YAWRATE);

    // throttle
    rc_command_.F.type = THROTTLE;
  }
}

bool CommandManager::stick_deviated(uint8_t channel)
{
  uint32_t now = RF_.board_.clock_millis();

  // if we are still in the lag time, return true
  if (now - rc_stick_override_[channel].last_override_time < (uint32_t)RF_.params_.get_param_int(PARAM_OVERRIDE_LAG_TIME))
  {
    return true;
  }
  else
  {
    if (abs(RF_.rc_.rc_stick(rc_stick_override_[channel].rc_channel)) > RF_.params_.get_param_float(PARAM_RC_OVERRIDE_DEVIATION))
    {
      rc_stick_override_[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}

bool CommandManager::do_roll_pitch_yaw_muxing(uint8_t channel)
{
  //Check if the override switch exists and is triggered, or if the sticks have deviated enough to trigger an override
  if ((RF_.rc_.rc_switch_mapped(RC::SWITCH_ATT_OVERRIDE) && RF_.rc_.rc_switch(RC::SWITCH_ATT_OVERRIDE)) || stick_deviated(channel))
  {
    rc_override_ = true;
  }
  else // Otherwise only have RC override if the offboard channel is inactive
  {
    if (muxes[channel].onboard->active)
    {
      rc_override_ = false;
    }
    else
    {
      rc_override_ = true;
    }
  }
  // set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[channel].combined = rc_override_ ? *muxes[channel].rc : *muxes[channel].onboard;
  return rc_override_;
}

bool CommandManager::do_throttle_muxing(void)
{
  // Check if the override switch exists and is triggered
  if (RF_.rc_.rc_switch_mapped(RC::SWITCH_THROTTLE_OVERRIDE) && RF_.rc_.rc_switch(RC::SWITCH_THROTTLE_OVERRIDE))
  {
    rc_override_ = true;
  }
  else // Otherwise check if the offboard throttle channel is active, if it isn't, have RC override
  {
    if (muxes[MUX_F].onboard->active)
    {
      // Check if the parameter flag is set to have us always take the smaller throttle
      if (RF_.params_.get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
      {
        rc_override_ = (muxes[MUX_F].rc->value < muxes[MUX_F].onboard->value);
      }
      else
      {
        rc_override_ = false;
      }
    }
    else
    {
      rc_override_ = true;
    }
  }

  // Set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[MUX_F].combined = rc_override_ ? *muxes[MUX_F].rc : *muxes[MUX_F].onboard;
  return rc_override_;
}

bool CommandManager::rc_override_active()
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
  // Check for and apply failsafe command
  if (RF_.state_manager_.state().failsafe)
  {
    failsafe_command_.F.value = RF_.params_.get_param_float(PARAM_FAILSAFE_THROTTLE);
    combined_command_ = failsafe_command_;
  }

  else if (RF_.rc_.new_command())
  {
    // Read RC
    interpret_rc();

    // Check for offboard control timeout (100 ms)
    if (RF_.board_.clock_micros() > offboard_command_.stamp_us + 100000)
    {
      // If it has been longer than 100 ms, then disable the offboard control
      offboard_command_.F.active = false;
      offboard_command_.x.active = false;
      offboard_command_.y.active = false;
      offboard_command_.z.active = false;
    }

    // Perform muxing
    rc_override_ = false;
    for (uint8_t i = MUX_X; i <= MUX_Z; i++)
    {
      do_roll_pitch_yaw_muxing(i);
    }
    do_throttle_muxing();

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
  return true;
}


}
