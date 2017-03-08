#include <stdbool.h>

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

mux_t muxes[4] =
{
  {&_rc_control.x, &_offboard_control.x, &_combined_control.x},
  {&_rc_control.y, &_offboard_control.y, &_combined_control.y},
  {&_rc_control.z, &_offboard_control.z, &_combined_control.z},
  {&_rc_control.F, &_offboard_control.F, &_combined_control.F}
};

void do_muxing(uint8_t mux_channel)
{
  mux_t* mux_ptr = &(muxes[mux_channel]);
  if(mux_ptr->rc->active)
  {
    (*mux_ptr->combined) = (*mux_ptr->rc);
  }
  else if (mux_ptr->onboard->active)
  {
    (*mux_ptr->combined) = (*mux_ptr->onboard);
  }
  else
  {
    // Default to RC if neither is active
    (*mux_ptr->combined) = (*mux_ptr->rc);
    mux_ptr->combined->active = true;
  }
}

void do_min_throttle_muxing()
{
  if (_offboard_control.F.active)
  {
    if (_rc_control.F.type == THROTTLE && _offboard_control.F.type == THROTTLE)
    {
      _combined_control.F.value = (_rc_control.F.value > _offboard_control.F.value) ?
                                  _offboard_control.F.value : _rc_control.F.value;
      _combined_control.F.type = THROTTLE;
      _combined_control.F.active = true;
    }
    else
    {
      // I'm still not quite sure how to handle the mixed altitude/throttle cases
      // for now, just pass the rc along.  I expect that what we really need to do
      // is run the altitude controller here so we can compare throttle to throttle
      _combined_control.F = _rc_control.F;
    }
  }
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
    for (uint8_t i = 0; i < 4; i++)
    {
      if (i == MUX_F && get_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE))
      {
        do_min_throttle_muxing();
      }
      else
      {
        do_muxing(i);
      }
    }

    // Light to indicate override
    if (_rc_control.x.active || _rc_control.y.active || _rc_control.z.active || _rc_control.F.active)
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
