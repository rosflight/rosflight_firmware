#include <stdbool.h>
#include <stdlib.h>

#include "mavlink_util.h"

#include "board.h"
#include "rc.h"
#include "mux.h"
#include "param.h"
#include "mode.h"

control_t _rc_control;
control_t _offboard_control;
control_t _combined_control;

control_t _failsafe_control =
{
  {true, ANGLE, 0.0},
  {true, ANGLE, 0.0},
  {true, RATE, 0.0},
  {true, THROTTLE, 0.0}
};

typedef enum
{
  ATT_MODE_RATE,
  ATT_MODE_ANGLE
} att_mode_t;

typedef enum
{
  THROTTLE_MODE_THROTTLE,
  THROTTLE_MODE_ALTITUDE
} throttle_mode_t;

typedef enum
{
  MUX_X,
  MUX_Y,
  MUX_Z,
  MUX_F,
} mux_channel_t;

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

mux_t muxes[4] =
{
  {&_rc_control.x, &_offboard_control.x, &_combined_control.x},
  {&_rc_control.y, &_offboard_control.y, &_combined_control.y},
  {&_rc_control.z, &_offboard_control.z, &_combined_control.z},
  {&_rc_control.F, &_offboard_control.F, &_combined_control.F}
};

static void interpret_rc(void)
{
  // get initial, unscaled RC values
  _rc_control.x.value = rc_stick(RC_STICK_X);
  _rc_control.y.value = rc_stick(RC_STICK_Y);
  _rc_control.z.value = rc_stick(RC_STICK_Z);
  _rc_control.F.value = rc_stick(RC_STICK_F);

  // determine control mode for each channel and scale command values accordingly
  if (get_param_int(PARAM_FIXED_WING))
  {
    _rc_control.x.type = PASSTHROUGH;
    _rc_control.y.type = PASSTHROUGH;
    _rc_control.z.type = PASSTHROUGH;
    _rc_control.F.type = THROTTLE;
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
    _rc_control.z.type = THROTTLE;

    /* TODO use this logic once altitude hold is implemented:
    if (rc_switch_mapped(RC_SWITCH_THROTTLE_TYPE))
    {
      _rc_control.z.type = rc_switch(RC_SWITCH_THROTTLE_TYPE) ? ALTITUDE : THROTTLE;
    }
    else
    {
      _rc_control.z.type = (get_param_int(PARAM_RC_THROTTLE_MODE) == THROTTLE_MODE_ALTITUDE) ? ALTITUDE : THROTTLE;
    }
    */

    // TODO run altitude controller here if in ALTITUDE mode, then result is in THROTTLE mode?
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
  {
    if (abs(rc_stick(rc_stick_override[channel].rc_channel)) > get_param_float(PARAM_RC_OVERRIDE_DEVIATION))
    {
      rc_stick_override[channel].last_override_time = now;
      return true;
    }
    return false;
  }
}

static bool do_roll_pitch_yaw_muxing(mux_channel_t channel)
{
  bool rc_override;

  if ((rc_switch_mapped(RC_SWITCH_ATT_OVERRIDE) && rc_switch(RC_SWITCH_ATT_OVERRIDE)) || stick_deviated(channel))
  {
    rc_override = true;
  }
  else
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

static bool do_throttle_muxing(void)
{
  // TODO this assumes that both RC and offboard are in THROTTLE mode

  bool rc_override;

  if (rc_switch_mapped(RC_SWITCH_THROTTLE_OVERRIDE) && rc_switch(RC_SWITCH_THROTTLE_OVERRIDE))
  {
    rc_override = true;
  }
  else
  {
    if (muxes[MUX_F].onboard->active)
    {
      if (get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
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

bool _new_command;

bool mux_inputs()
{
  if (!_new_command)
  {
    // we haven't received any new commands, so we shouldn't do anything
    return false;
  }

  // otherwise combine the new commands
  if (_armed_state & FAILSAFE)
  {
    _combined_control = _failsafe_control;
  }
  else
  {
    bool rc_override = false;
    for (mux_channel_t i = MUX_X; i <= MUX_Z; i++)
    {
      rc_override = rc_override || do_roll_pitch_yaw_muxing(i);
    }
    rc_override = rc_override || do_throttle_muxing();

    // Light to indicate override
    if (rc_override)
    {
      led0_on();
    }
    else
    {
      led0_off();
    }
  }

  // reset the new command flag
  _new_command = false;
  return true;
}
