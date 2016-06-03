#include <stdbool.h>

#include "rc.h"
#include "mux.h"
#include "param.h"

control_t _rc_control;
control_t _offboard_control;
control_t _combined_control;

void mux_inputs()
{

  if(_rc_control.x.active)
  {
    _combined_control.x = _rc_control.x;
  }
  else if(_offboard_control.x.active)
  {
    _combined_control.x = _offboard_control.x;
  }
  else
  {
    // default to taking RC if neither is publishing
    _combined_control.x = _rc_control.x;
    _combined_control.x.active = true;
  }


  if(_rc_control.y.active)
  {
    _combined_control.y = _rc_control.y;
  }
  else if(_offboard_control.y.active)
  {
    _combined_control.y = _offboard_control.y;
  }
  else
  {
    // default to taking RC if neither is publishing
    _combined_control.y = _rc_control.y;
    _combined_control.y.active = true;
  }


  if(_rc_control.z.active)
  {
    _combined_control.z = _rc_control.z;
  }
  else if(_offboard_control.z.active)
  {
    _combined_control.z = _offboard_control.z;
  }
  else
  {
    _combined_control.z = _rc_control.z;
    _combined_control.z.active = true;
  }

  if(_offboard_control.F.active)
  {
    if ( _params.values[PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE])
    {
      if (_rc_control.F.type == THROTTLE && _offboard_control.F.type == THROTTLE)
      {
        _combined_control.F.value = (_rc_control.F.value > _offboard_control.F.value) ?  _offboard_control.F.value : _rc_control.F.value;
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
    else // offboard is active, but not min throttle
    {
      _combined_control.F = _offboard_control.F;
    }
  }
  else
  {
    _combined_control.F = _rc_control.F;
    _combined_control.F.active = true;
  }
}