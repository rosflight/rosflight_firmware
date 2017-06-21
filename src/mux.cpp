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

#include "mux.h"
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
  rc_stick_t rc_channel;
  uint32_t last_override_time;
} rc_stick_override_t;

rc_stick_override_t rc_stick_override[] =
{
  { RC_STICK_X, 0 },
  { RC_STICK_Y, 0 },
  { RC_STICK_Z, 0 }
};

typedef struct
{
  control_channel_t *rc;
  control_channel_t *onboard;
  control_channel_t *combined;
} mux_t;

void Mux::init(ROSflight* _rf)
{
  RF_ = _rf;
}

void Mux::interpret_rc(void)
{
  // get initial, unscaled RC values
  _rc_control.x.value = RF_->rc_.rc_stick(RC_STICK_X);
  _rc_control.y.value = RF_->rc_.rc_stick(RC_STICK_Y);
  _rc_control.z.value = RF_->rc_.rc_stick(RC_STICK_Z);
  _rc_control.F.value = RF_->rc_.rc_stick(RC_STICK_F);

  // determine control mode for each channel and scale command values accordingly
  if (RF_->params_.get_param_int(PARAM_FIXED_WING))
  {
    _rc_control.x.type = PASSTHROUGH;
    _rc_control.y.type = PASSTHROUGH;
    _rc_control.z.type = PASSTHROUGH;
  }
  else
  {
    // roll and pitch
    control_type_t roll_pitch_type;
    if (RF_->rc_.rc_switch_mapped(RC_SWITCH_ATT_TYPE))
    {
      roll_pitch_type = RF_->rc_.rc_switch(RC_SWITCH_ATT_TYPE) ? ANGLE : RATE;
    }
    else
    {
      roll_pitch_type = (RF_->params_.get_param_int(PARAM_RC_ATTITUDE_MODE) == ATT_MODE_RATE) ? RATE: ANGLE;
    }

    _rc_control.x.type = roll_pitch_type;
    _rc_control.y.type = roll_pitch_type;

    // Scale command to appropriate units
    switch (roll_pitch_type)
    {
    case RATE:
      _rc_control.x.value *= RF_->params_.get_param_float(PARAM_RC_MAX_ROLLRATE);
      _rc_control.y.value *= RF_->params_.get_param_float(PARAM_RC_MAX_PITCHRATE);
      break;
    case ANGLE:
      _rc_control.x.value *= RF_->params_.get_param_float(PARAM_RC_MAX_ROLL);
      _rc_control.y.value *= RF_->params_.get_param_float(PARAM_RC_MAX_PITCH);
    }

    // yaw
    _rc_control.z.type = RATE;
    _rc_control.z.value *= RF_->params_.get_param_float(PARAM_RC_MAX_YAWRATE);

    // throttle
    _rc_control.F.type = THROTTLE;
  }
}

bool Mux::stick_deviated(uint8_t channel)
{
  uint32_t now = RF_->board_->clock_millis();

  // if we are still in the lag time, return true
  if (now - rc_stick_override[channel].last_override_time < (uint32_t)RF_->params_.get_param_int(PARAM_OVERRIDE_LAG_TIME))
  {
    return true;
  }
  else
  {
    if (abs(RF_->rc_.rc_stick(rc_stick_override[channel].rc_channel)) > RF_->params_.get_param_float(PARAM_RC_OVERRIDE_DEVIATION))
    {
      rc_stick_override[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}

bool Mux::do_roll_pitch_yaw_muxing(uint8_t channel)
{
  //Check if the override switch exists and is triggered, or if the sticks have deviated enough to trigger an override
  if ((RF_->rc_.rc_switch_mapped(RC_SWITCH_ATT_OVERRIDE) && RF_->rc_.rc_switch(RC_SWITCH_ATT_OVERRIDE)) || stick_deviated(channel))
  {
    rc_override = true;
  }
  else // Otherwise only have RC override if the offboard channel is inactive
  {
    if (muxes[channel].onboard->active)
    {
      rc_override = false;
    }
    else
    {
      rc_override = true;
    }
  }
  // set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[channel].combined = rc_override ? *muxes[channel].rc : *muxes[channel].onboard;
  return rc_override;
}

bool Mux::do_throttle_muxing(void)
{
  // Check if the override switch exists and is triggered
  if (RF_->rc_.rc_switch_mapped(RC_SWITCH_THROTTLE_OVERRIDE) && RF_->rc_.rc_switch(RC_SWITCH_THROTTLE_OVERRIDE))
  {
    rc_override = true;
  }
  else // Otherwise check if the offboard throttle channel is active, if it isn't, have RC override
  {
    if (muxes[MUX_F].onboard->active)
    {
      // Check if the parameter flag is set to have us always take the smaller throttle
      if (RF_->params_.get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
      {
        rc_override = (muxes[MUX_F].rc->value < muxes[MUX_F].onboard->value);
      }
      else
      {
        rc_override = false;
      }
    }
    else
    {
      rc_override = true;
    }
  }

  // Set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[MUX_F].combined = rc_override ? *muxes[MUX_F].rc : *muxes[MUX_F].onboard;
  return rc_override;
}

bool Mux::rc_override_active()
{
  return rc_override;
}

bool Mux::offboard_control_active()
{
  for (int i = 0; i < 4; i++)
  {
    if (muxes[i].onboard->active)
      return true;
  }
  return false;
}

void Mux::signal_new_command()
{
  new_command = true;
}

bool Mux::mux_inputs()
{
  // Check for and apply failsafe command
  if (RF_->fsm_.in_failsafe())
  {
    _failsafe_control.F.value = RF_->params_.get_param_float(PARAM_FAILSAFE_THROTTLE);
    _combined_control = _failsafe_control;
  }

  else if (RF_->rc_.new_command())
  {
    // Read RC
    interpret_rc();

    // Check for offboard control timeout (100 ms)
    if (RF_->board_->clock_micros() > _offboard_control.stamp_us + 100000)
    {
      // If it has been longer than 100 ms, then disable the offboard control
      _offboard_control.F.active = false;
      _offboard_control.x.active = false;
      _offboard_control.y.active = false;
      _offboard_control.z.active = false;
    }


    // Perform muxing
    rc_override = false;
    for (uint8_t i = MUX_X; i <= MUX_Z; i++)
    {
      do_roll_pitch_yaw_muxing(i);
    }
    do_throttle_muxing();

    // Light to indicate override
    if (rc_override)
    {
      RF_->board_->led0_on();
    }
    else
    {
      RF_->board_->led0_off();
    }
  }
  return true;
}


}
