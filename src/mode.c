#include <stdint.h>
#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "rc.h"
#include "param.h"
#include "mavlink_util.h"
#include "mux.h"
#include "sensors.h"

#include "mode.h"

#include "mavlink_log.h"


armed_state_t _armed_state;

void init_mode(void)
{
  _armed_state = DISARMED;
}

bool arm(void)
{
  static bool started_gyro_calibration = false;
  if (!started_gyro_calibration && _armed_state == DISARMED)
  {
    start_gyro_calibration();
    started_gyro_calibration = true;
    return false;
  }
  else if (gyro_calibration_complete())
  {
    started_gyro_calibration = false;
    _armed_state = ARMED;
    LED1_ON;
    return true;
  }
  return false;
}

void disarm(void)
{
  _armed_state = DISARMED;
  LED1_OFF;
}

/// TODO: Be able to tell if the RC has become disconnected during flight
bool check_failsafe(void)
{
  for(int8_t i = 0; i<get_param_int(PARAM_RC_NUM_CHANNELS); i++)
  {
    if(pwmRead(i) < 900 || pwmRead(i) > 2100)
    {
      if(_armed_state == ARMED || _armed_state == DISARMED)
      {
        _armed_state = FAILSAFE_DISARMED;
      }

      // blink LED
      static uint8_t count = 0;
      if(count > 25)
      {
        LED1_TOGGLE;
        count = 0;
      }
      count++;
      return true;
    }
  }

  // we got a valid RC measurement for all channels
  if(_armed_state == FAILSAFE_ARMED || _armed_state == FAILSAFE_DISARMED)
  {
    // return to appropriate mode
    _armed_state = (_armed_state == FAILSAFE_ARMED) ? ARMED : DISARMED;
  }
  return false;
}


bool check_mode(uint64_t now)
{
  static uint64_t prev_time = 0;
  static uint32_t time_sticks_have_been_in_arming_position = 0;

  // see it has been at least 20 ms
  uint32_t dt = now-prev_time;
  if (dt < 20000)
  {
    return false;
  }

  // if it has, then do stuff
  prev_time = now;

  // check for failsafe mode
  if(check_failsafe())
  {
    return true;
  }
  else
  {
    // check for arming switch
    if (get_param_int(PARAM_ARM_STICKS))
    {
      if (_armed_state == DISARMED)
      {
        // if left stick is down and to the right
        if (pwmRead(get_param_int(PARAM_RC_F_CHANNEL)) < get_param_int(PARAM_RC_F_BOTTOM) + get_param_int(PARAM_ARM_THRESHOLD)
            && pwmRead(get_param_int(PARAM_RC_Z_CHANNEL)) > (get_param_int(PARAM_RC_Z_CENTER) + get_param_int(PARAM_RC_Z_RANGE)/2)
            - get_param_int(PARAM_ARM_THRESHOLD))
        {
          time_sticks_have_been_in_arming_position += dt;
        }
        else
        {
          time_sticks_have_been_in_arming_position = 0;
        }
        if (time_sticks_have_been_in_arming_position > 500000)
        {
          if(arm())
            time_sticks_have_been_in_arming_position = 0;
        }
      }
      else // _armed_state is ARMED
      {
        // if left stick is down and to the left
        if (pwmRead(get_param_int(PARAM_RC_F_CHANNEL)) < get_param_int(PARAM_RC_F_BOTTOM) +
            get_param_int(PARAM_ARM_THRESHOLD)
            && pwmRead(get_param_int(PARAM_RC_Z_CHANNEL)) < (get_param_int(PARAM_RC_Z_CENTER) - get_param_int(PARAM_RC_Z_RANGE)/2)
            + get_param_int(PARAM_ARM_THRESHOLD))
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
      if (rc_switch(get_param_int(PARAM_ARM_CHANNEL)))
      {
        arm();
      }
      else
      {
        disarm();
      }
    }
  }
  return true;
}
