#include <stdint.h>
#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "param.h"
#include "rc.h"

rc_control_t _rc_commands;

void init_rc()
{
  _rc_commands.x_type = RC_ANGLE;
  _rc_commands.y_type = RC_ANGLE;
  _rc_commands.z_type = RC_ANGLE;
  _rc_commands.F_type = RC_THROTTLE;

  _rc_commands.x = 0;
  _rc_commands.y = 0;
  _rc_commands.z = 0;
  _rc_commands.F = 0;
}

void receive_rc()
{
  static uint32_t time_of_last_stick_deviation = 0;

  // Start by getting Values from RC
  _rc_commands.x = pwmRead(_params.values[PARAM_RC_X_CHANNEL]);
  _rc_commands.y = pwmRead(_params.values[PARAM_RC_Y_CHANNEL]);
  _rc_commands.z = pwmRead(_params.values[PARAM_RC_Z_CHANNEL]);
  _rc_commands.F = pwmRead(_params.values[PARAM_RC_F_CHANNEL]);

  uint32_t now = micros();

  // Figure out the desired control type from the switches and params
  rc_control_type_t attitude_type = (pwmRead(_params.values[PARAM_RC_ATT_CONTROL_TYPE_CHANNEL]) > 1500) ? RC_ANGLE : RC_RATE;
  rc_control_type_t F_type = (pwmRead(_params.values[PARAM_RC_F_CONTROL_TYPE_CHANNEL]) > 1500) ? RC_ALTITUDE : RC_THROTTLE;
  if ( _params.values[PARAM_FIXED_WING] )
  {
    // for using fixedwings
    attitude_type = RC_PASSTHROUGH;
    F_type = RC_THROTTLE;
  }

  // Set flags for attitude channels
  if (pwmRead(_params.values[PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL]) > 1500 || now - time_of_last_stick_deviation < _params.values[PARAM_OVERRIDE_LAG_TIME])
  {
    // Pilot is in full control
    _rc_commands.x_type = attitude_type;
    _rc_commands.y_type = attitude_type;
    _rc_commands.z_type = attitude_type;
  }
  else
  {
    // Check for stick deviation
    _rc_commands.x_type = (abs(_rc_commands.x - _params.values[PARAM_RC_X_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION]) ? attitude_type : RC_OFFBOARD;
    _rc_commands.y_type = (abs(_rc_commands.y - _params.values[PARAM_RC_Y_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION]) ? attitude_type : RC_OFFBOARD;
    _rc_commands.z_type = (abs(_rc_commands.z - _params.values[PARAM_RC_Z_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION]) ? RC_RATE : RC_OFFBOARD;
    time_of_last_stick_deviation = now;
  }

  // Set flags for throttle channel
  if (pwmRead(_params.values[PARAM_RC_THROTTLE_OVERRIDE_CHANNEL]) > 1500)
  {
    // RC Pilot is in full control
    _rc_commands.F_type = F_type;
  }
  else
  {
    // Check for the min_throttle case
    if (_params.values[PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE])
    {
      if ( F_type == RC_ALTITUDE )
      {
        _rc_commands.F_type =  RC_MIN_THROTTLE_ALTITUDE ;
      }
      else
      {
        _rc_commands.F_type = RC_MIN_THROTTLE;
      }
    }
    else
    {
      // entirely offboard control
      _rc_commands.F_type = RC_PASSTHROUGH;
    }
  }
}


