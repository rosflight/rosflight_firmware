#include <stdint.h>
#include <stdbool.h>

#include "board.h"
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
  if (!started_gyro_calibration && _armed_state & DISARMED)
  {
    start_gyro_calibration();
    started_gyro_calibration = true;
    return false;
  }
  else if (gyro_calibration_complete())
  {
    started_gyro_calibration = false;
    _armed_state |= ARMED;
    led1_on();
    return true;
  }
  return false;
}

void disarm(void)
{
  _armed_state &= ~(ARMED);
  led1_off();
}

bool check_failsafe(void)
{
  if (pwm_lost())
  {
    // Set the FAILSAFE bit
    _armed_state |= FAILSAFE;
    return true;
  }

  else
  {
    for (int8_t i = 0; i<get_param_int(PARAM_RC_NUM_CHANNELS); i++)
    {
      if(pwm_read(i) < 900 || pwm_read(i) > 2100)
      {
        _armed_state |= FAILSAFE;

        // blink LED
        static uint8_t count = 0;
        if (count > 25)
        {
          led1_toggle();
          count = 0;
        }
        count++;
        return true;
      }
    }

    // we got a valid RC measurement for all channels and pwm is active
    // Clear the FAILSAFE bit
    _armed_state &= ~(FAILSAFE);
    return false;
  }
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
  if (check_failsafe())
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
        if (rc_low(RC_STICK_F) && rc_high(RC_STICK_Z))
        {
          time_sticks_have_been_in_arming_position += dt;
        }
        else
        {
          time_sticks_have_been_in_arming_position = 0;
        }
        if (time_sticks_have_been_in_arming_position > 500000)
        {
          if (arm())
            time_sticks_have_been_in_arming_position = 0;
        }
      }
      else // _armed_state is ARMED
      {
        // if left stick is down and to the left
        if (rc_low(RC_STICK_F) && rc_low(RC_STICK_Z))
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
        if (_armed_state == DISARMED)
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
