#include <stdint.h>
#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "param.h"
#include "rc.h"

rc_control_t _rc_commands;

void init_rc()
{
  _rc_control.x.type = ANGLE;
  _rc_control.y.type = ANGLE;
  _rc_control.z.type = RATE;
  _rc_control.F.type = THROTTLE;

  _rc_control.x.value = 0;
  _rc_control.y.value = 0;
  _rc_control.z.value = 0;
  _rc_control.F.value = 0;
}

void receive_rc()
{
  static uint32_t time_of_last_stick_deviation = 0;

  // Start by getting Values from RC
  _rc_control.x.value = pwmRead(_params.values[PARAM_RC_X_CHANNEL]);
  _rc_control.y.value = pwmRead(_params.values[PARAM_RC_Y_CHANNEL]);
  _rc_control.z.value = pwmRead(_params.values[PARAM_RC_Z_CHANNEL]);
  _rc_control.F.value = pwmRead(_params.values[PARAM_RC_F_CHANNEL]);


  // Get timestamp for deadband control lag
  uint32_t now = micros();


  // Figure out the desired control type from the switches and params
  control_type_t rc_attitude_type = (pwmRead(_params.values[PARAM_RC_ATT_CONTROL_TYPE_CHANNEL]) > 1500) ? ANGLE : RATE;
  control_type_t rc_F_type = (pwmRead(_params.values[PARAM_RC_F_CONTROL_TYPE_CHANNEL]) > 1500) ? ALTITUDE : THROTTLE;
  if ( _params.values[PARAM_FIXED_WING] )
  {
    // for using fixedwings
    rc_attitude_type = PASSTHROUGH;
    rc_F_type = THROTTLE;
  }


  // Set flags for attitude channels
  if (pwmRead(_params.values[PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL]) > 1500 || now - time_of_last_stick_deviation < _params.values[PARAM_OVERRIDE_LAG_TIME])
  {
    // Pilot is in full control
    _rc_control.x.active = true;
    _rc_control.y.active = true;
    _rc_control.z.active = true;
  }
  else
  {
    // Check for stick deviation - if so, then that channel is active
    // Otherwise let onboard computer have control
    _rc_control.x.active = abs(_rc_commands.x - _params.values[PARAM_RC_X_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION];
    _rc_control.y.active = abs(_rc_commands.y - _params.values[PARAM_RC_Y_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION];
    _rc_control.z.active = abs(_rc_commands.z - _params.values[PARAM_RC_Z_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION];
    time_of_last_stick_deviation = now;
  }


  // Set flags for throttle channel
  if (pwmRead(_params.values[PARAM_RC_THROTTLE_OVERRIDE_CHANNEL]) > 1500)
  {
    // RC Pilot is in full control
    _rc_control.F.active = true;
  }
  else
  {
    // Onboard Control
    _rc_control.F.active = false;
  }
}


