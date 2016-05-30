#include <stdint.h>
#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "param.h"
#include "rc.h"

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

static void convertPWMtoRad(){
  // Get Roll control command out of RC
  if ( _rc_control.x.type == ANGLE)
  {
    _rc_control.x.value = ((pwmRead(_params.values[PARAM_RC_X_CHANNEL]) - _params.values[PARAM_RC_X_CENTER])
        *_params.values[PARAM_RC_MAX_ROLL_MRAD])/_params.values[PARAM_RC_X_RANGE];
  }
  else if (_rc_control.x.type == RATE)
  {
    _rc_control.x.value = ((pwmRead(_params.values[PARAM_RC_X_CHANNEL]) - _params.values[PARAM_RC_X_CENTER])
        *_params.values[PARAM_RC_MAX_ROLLRATE_MRAD_S])/_params.values[PARAM_RC_X_RANGE];
  }
  else
  {
    _rc_control.x.value = pwmRead(_params.values[PARAM_RC_X_CHANNEL]);
  }

  // Get Pitch control command out of RC
  if ( _rc_control.y.type == ANGLE)
  {
    _rc_control.y.value = ((pwmRead(_params.values[PARAM_RC_Y_CHANNEL]) - _params.values[PARAM_RC_Y_CENTER])
        *_params.values[PARAM_RC_MAX_PITCH_MRAD])/_params.values[PARAM_RC_Y_RANGE];
  }
  else if (_rc_control.y.type == RATE)
  {
    _rc_control.y.value = ((pwmRead(_params.values[PARAM_RC_Y_CHANNEL]) - _params.values[PARAM_RC_Y_CENTER])
        *_params.values[PARAM_RC_MAX_PITCHRATE_MRAD_S])/_params.values[PARAM_RC_Y_RANGE];
  }
  else
  {
    _rc_control.x.value = pwmRead(_params.values[PARAM_RC_Y_CHANNEL]);
  }

  // Get the Yaw control command type out of RC
  if ( _rc_control.z.type == RATE)
  {
    _rc_control.z.value = ((pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) - _params.values[PARAM_RC_Z_CENTER])
        *_params.values[PARAM_RC_MAX_YAWRATE_MRAD_S])/_params.values[PARAM_RC_Z_RANGE];
  }
  else
  {
    _rc_control.x.value = pwmRead(_params.values[PARAM_RC_Z_CHANNEL]);
  }

  // Finally, the throttle command
  _rc_control.F.value = (pwmRead(_params.values[PARAM_RC_F_CHANNEL]) - _params.values[PARAM_RC_F_BOTTOM])
      * 1000 / _params.values[PARAM_RC_F_RANGE];
}


void receive_rc()
{
  static uint32_t time_of_last_stick_deviation = 0;
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

  // Interpret PWM Values from RC
  convertPWMtoRad();

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
    _rc_control.x.active = abs(pwmRead(_params.values[PARAM_RC_X_CHANNEL]) - _params.values[PARAM_RC_X_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION];
    _rc_control.y.active = abs(pwmRead(_params.values[PARAM_RC_Y_CHANNEL]) - _params.values[PARAM_RC_Y_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION];
    _rc_control.z.active = abs(pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) - _params.values[PARAM_RC_Z_CENTER]) >  _params.values[PARAM_RC_OVERRIDE_DEVIATION];
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
    // Onboard Control - min throttle Checking will be done in mux and in the controller.
    _rc_control.F.active = false;
  }


  // Convert PWM inputs to rad or rads/
}


