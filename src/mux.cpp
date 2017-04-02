#include <stdbool.h>
#include <stdlib.h>

#include "mux.h"

namespace rosflight
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

rc_stick_override_t rc_stick_override[] = {
  { RC_STICK_X, 0 },
  { RC_STICK_Y, 0 },
  { RC_STICK_Z, 0 }
};

typedef struct
{
  control_channel_t* rc;
  control_channel_t* onboard;
  control_channel_t* combined;
} mux_t;

void Mux::init(Mode *_fsm, Params *_params, Board *_board, RC *_rc)
{
  rc = _rc;
  params = _params;
  board = _board;
  fsm = _fsm;
}

void Mux::interpret_rc(void)
{
  // get initial, unscaled RC values
  _rc_control.x.value = rc->rc_stick(RC_STICK_X);
  _rc_control.y.value = rc->rc_stick(RC_STICK_Y);
  _rc_control.z.value = rc->rc_stick(RC_STICK_Z);
  _rc_control.F.value = rc->rc_stick(RC_STICK_F);

  // determine control mode for each channel and scale command values accordingly
  if (params->get_param_int(PARAM_FIXED_WING))
  {
    _rc_control.x.type = PASSTHROUGH;
    _rc_control.y.type = PASSTHROUGH;
    _rc_control.z.type = PASSTHROUGH;
  }
  else
  {
    // roll and pitch
    control_type_t roll_pitch_type;
    if (rc->rc_switch_mapped(RC_SWITCH_ATT_TYPE))
    {
      roll_pitch_type = rc->rc_switch(RC_SWITCH_ATT_TYPE) ? ANGLE : RATE;
    }
    else
    {
      roll_pitch_type = (params->get_param_int(PARAM_RC_ATTITUDE_MODE) == ATT_MODE_RATE) ? RATE: ANGLE;
    }

    _rc_control.x.type = roll_pitch_type;
    _rc_control.y.type = roll_pitch_type;

    // Scale command to appropriate units
    switch (roll_pitch_type)
    {
      case RATE:
        _rc_control.x.value *= params->get_param_float(PARAM_RC_MAX_ROLLRATE);
        _rc_control.y.value *= params->get_param_float(PARAM_RC_MAX_PITCHRATE);
        break;
      case ANGLE:
        _rc_control.x.value *= params->get_param_float(PARAM_RC_MAX_ROLL);
        _rc_control.y.value *= params->get_param_float(PARAM_RC_MAX_PITCH);
    }

    // yaw
    _rc_control.z.type = RATE;
    _rc_control.z.value *= params->get_param_float(PARAM_RC_MAX_YAWRATE);

    // throttle
    _rc_control.F.type = THROTTLE;
  }
}

bool Mux::stick_deviated(mux_channel_t channel)
{
  uint32_t now = board->clock_millis();

  // if we are still in the lag time, return true
  if (now - rc_stick_override[channel].last_override_time < (uint32_t)params->get_param_int(PARAM_OVERRIDE_LAG_TIME))
  {
    return true;
  }
  else
  {
    if (abs(rc->rc_stick(rc_stick_override[channel].rc_channel)) > params->get_param_float(PARAM_RC_OVERRIDE_DEVIATION))
    {
      rc_stick_override[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}

bool Mux::do_roll_pitch_yaw_muxing(mux_channel_t channel)
{
  bool rc_override;

  if ((rc->rc_switch_mapped(RC_SWITCH_ATT_OVERRIDE) && rc->rc_switch(RC_SWITCH_ATT_OVERRIDE)) || stick_deviated(channel))
  {
    rc_override = true;
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
    if (muxes[channel].onboard->active)
    {
      rc_override = false;
    }
    else
    {
      rc_override = true;
    }
  }

  *muxes[channel].combined = rc_override ? *muxes[channel].rc : *muxes[channel].onboard;
  return rc_override;
}

bool Mux::do_throttle_muxing(void)
{
  bool rc_override;

  if (rc->rc_switch_mapped(RC_SWITCH_THROTTLE_OVERRIDE) && rc->rc_switch(RC_SWITCH_THROTTLE_OVERRIDE))
  {
    rc_override = true;
  }
  else
  {
    if (muxes[MUX_F].onboard->active)
    {
      if (params->get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
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

  *muxes[MUX_F].combined = rc_override ? *muxes[MUX_F].rc : *muxes[MUX_F].onboard;
  return rc_override;
}

void Mux::signal_new_command()
{
  new_command = true;
}

bool Mux::mux_inputs()
{
  // otherwise combine the new commands
  if (fsm->in_failsafe())
  {
    _combined_control = _failsafe_control;
  }

  else if (rc->new_command())
  {
    // Read RC
    interpret_rc();

    // Perform muxing
    bool rc_override = false;
    for (uint8_t i = MUX_X; i <= MUX_Z; i++)
    {
      rc_override |= do_roll_pitch_yaw_muxing((mux_channel_t)i);
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
  return true;
}


}
