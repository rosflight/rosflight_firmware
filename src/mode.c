#include <stdint.h>
#include <stdbool.h>

#include "rc.h"
#include "param.h"
#include "mavlink_util.h"

#include <breezystm32/breezystm32.h>

#include "mode.h"


armed_state_t _armed_state;

void init_mode(void)
{
  _armed_state = DISARMED;
}

void arm(void)
{
  _armed_state = ARMED;
  LED1_ON;
}

void disarm(void)
{
  _armed_state = DISARMED;
  LED1_OFF;
}

bool check_mode(uint32_t now)
{
  static uint32_t prev_time = 0;
  static uint32_t time_sticks_have_been_in_arming_position = 0;

  // see it has been at least 20 ms
  uint32_t dt = now-prev_time;
  if (dt < 20000)
  {
    return false;
  }

  // if it has, then do stuff
  prev_time = now;

  // check for arming switch
  if (_params.values[PARAM_ARM_STICKS])
  {
    if (_armed_state == DISARMED)
    {
      // if left stick is down and to the right
      if (pwmRead(_params.values[PARAM_RC_F_CHANNEL]) < _params.values[PARAM_RC_F_BOTTOM] +
          _params.values[PARAM_ARM_THRESHOLD]
          && pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) > (_params.values[PARAM_RC_Z_CENTER] + _params.values[PARAM_RC_Z_RANGE]/2)
          - _params.values[PARAM_ARM_THRESHOLD])
      {
        time_sticks_have_been_in_arming_position += dt;
      }
      else
      {
        time_sticks_have_been_in_arming_position = 0;
      }
      if (time_sticks_have_been_in_arming_position > 500000)
      {
        arm();
        time_sticks_have_been_in_arming_position = 0;
      }
    }
    else // _armed_state is ARMED
    {
      // if left stick is down and to the left
      if (pwmRead(_params.values[PARAM_RC_F_CHANNEL]) < _params.values[PARAM_RC_F_BOTTOM] +
          _params.values[PARAM_ARM_THRESHOLD]
          && pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) < (_params.values[PARAM_RC_Z_CENTER]-_params.values[PARAM_RC_Z_RANGE]/2)
          + _params.values[PARAM_ARM_THRESHOLD])
      {
        time_sticks_have_been_in_arming_position += dt;
      }
      else
      {
        time_sticks_have_been_in_arming_position = 0;
      }
      if (time_sticks_have_been_in_arming_position > 500000)
      {
        disarm();
        time_sticks_have_been_in_arming_position = 0;
      }
    }
  }
  else
  {
    if (pwmRead(_params.values[PARAM_ARM_CHANNEL]) > 1500)
    {
      arm();
    }
    else
    {
      disarm();
    }
  }
  return true;
}
