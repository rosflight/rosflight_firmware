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
#include "mux.h"


namespace rosflight {

void Mux::init(Arming_FSM* _fsm, Params* _params, Board* _board)
{
  fsm = _fsm;
  params = _params;
  board = _board;
  _new_command = false;
}

void Mux::do_muxing(uint8_t mux_channel)
{
  // get initial, unscaled RC values
  _rc_control.x.value = rc_stick(RC_STICK_X);
  _rc_control.y.value = rc_stick(RC_STICK_Y);
  _rc_control.z.value = rc_stick(RC_STICK_Z);
  _rc_control.F.value = rc_stick(RC_STICK_F);

  // determine control mode for each channel and scale command values accordingly
  if (get_param_int(PARAM_FIXED_WING)) //Fixed wing aircraft have no command scaling or PID control
  {
    _rc_control.x.type = PASSTHROUGH;
    _rc_control.y.type = PASSTHROUGH;
    _rc_control.z.type = PASSTHROUGH;
  }
  else
  {
    // roll and pitch
    control_type_t roll_pitch_type;
    if (rc_switch_mapped(RC_SWITCH_ATT_TYPE))
    {
      roll_pitch_type = rc_switch(RC_SWITCH_ATT_TYPE) ? ANGLE : RATE;
    }
    else
    {
      roll_pitch_type = (get_param_int(PARAM_RC_ATTITUDE_MODE) == ATT_MODE_RATE) ? RATE: ANGLE;
    }

    _rc_control.x.type = roll_pitch_type;
    _rc_control.y.type = roll_pitch_type;

    // Scale command to appropriate units
    switch (roll_pitch_type)
    {
    case RATE:
      _rc_control.x.value *= get_param_float(PARAM_RC_MAX_ROLLRATE);
      _rc_control.y.value *= get_param_float(PARAM_RC_MAX_PITCHRATE);
      break;
    case ANGLE:
      _rc_control.x.value *= get_param_float(PARAM_RC_MAX_ROLL);
      _rc_control.y.value *= get_param_float(PARAM_RC_MAX_PITCH);
    }

    // yaw
    _rc_control.z.type = RATE;
    _rc_control.z.value *= get_param_float(PARAM_RC_MAX_YAWRATE);

    // throttle
    _rc_control.F.type = THROTTLE;
  }
}

static bool stick_deviated(mux_channel_t channel)
{
  uint32_t now = clock_millis();

  // if we are still in the lag time, return true
  if (now - rc_stick_override[channel].last_override_time < (uint32_t)get_param_int(PARAM_OVERRIDE_LAG_TIME))
  {
    return true;
  }
  else
  { //check if the RC value for this channel has moved from center enough to trigger a RC override
    if (fabs(rc_stick(rc_stick_override[channel].rc_channel)) > get_param_float(PARAM_RC_OVERRIDE_DEVIATION))
    {
      rc_stick_override[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}

static bool do_roll_pitch_yaw_muxing(mux_channel_t channel)
{
  //Check if the override switch exists and is triggered, or if the sticks have deviated enough to trigger an override
  if ((rc_switch_mapped(RC_SWITCH_ATT_OVERRIDE) && rc_switch(RC_SWITCH_ATT_OVERRIDE)) || stick_deviated(channel))
  {
    rc_override = true;
  }
  else //Otherwise only have RC override if the offboard channel is inactive
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

  //set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[channel].combined = rc_override ? *muxes[channel].rc : *muxes[channel].onboard;
  return rc_override;
}

void Mux::do_min_throttle_muxing()
{
  bool rc_override;

  //Check if the override switch exists and is triggered
  if (rc_switch_mapped(RC_SWITCH_THROTTLE_OVERRIDE) && rc_switch(RC_SWITCH_THROTTLE_OVERRIDE))
  {
    rc_override = true;
  }
  else //Otherwise check if the offboard throttle channel is active, if it isn't, have RC override
  {
    if (muxes[MUX_F].onboard->active)
    {
      if (get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE)) //Check if the parameter flag is set to have us always take the smaller throttle
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

  //set the combined channel output depending on whether RC is overriding for this channel or not
  *muxes[MUX_F].combined = rc_override ? *muxes[MUX_F].rc : *muxes[MUX_F].onboard;
  return rc_override;
}

bool Mux::mux_inputs()
{
  return rc_override;
}

bool offboard_control_active()
{
  for (int i = 0; i < 4; i++)
  {
    if(muxes[i].onboard->active)
      return true;
  }
  return false;
}

  if (fsm->_armed_state == DISARMED_FAILSAFE || fsm->_armed_state == ARMED_FAILSAFE)
  {
    _failsafe_control.F.value = get_param_float(PARAM_FAILSAFE_THROTTLE);
    _combined_control = _failsafe_control;
  }

  else if (!_new_command)
  {
    // we haven't received any new commands, so we shouldn't do anything
    return false;
  }

  // Otherwise, combine commands
  else
  {
    // Read RC
    interpret_rc();

    // Check for offboard control timeout (100 ms)
    if (clock_micros() > _offboard_control_time + 100000)
    {
      if (i == MUX_F && params->get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
      {
        do_min_throttle_muxing();
      }
      else
      {
        do_muxing(i);
      }
    }
    rc_override |= do_throttle_muxing();

    // Light to indicate override
    if (rc_override)
    {
      board->led0_on();
    }
    else
    {
      board->led0_off();
    }
  }

  // reset the new command flag
  _new_command = false;
  return true;
}

}
