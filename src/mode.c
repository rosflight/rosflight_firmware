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
  _armed_state = 0x00;
}

bool arm(void)
{
  static bool started_gyro_calibration = false;
  if (!started_gyro_calibration && !(_armed_state & ARMED))
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

  bool failsafe = false;

  if (pwm_lost())
  {
    // Set the FAILSAFE bit
    failsafe = true;
  }
  else
  {
    // go into failsafe if we get an invalid RC command for any channel
    for (int8_t i = 0; i<get_param_int(PARAM_RC_NUM_CHANNELS); i++)
    {
      if(pwm_read(i) < 900 || pwm_read(i) > 2100)
      {
        failsafe = true;
      }
    }
  }

  if (failsafe)
  {
    // blink LED to let the user know we are in failsafe
    static uint8_t count = 0;
    if (count > 25)
    {
      led1_toggle();
      count = 0;
    }
    count++;

    // Set the FAILSAFE bit
    _armed_state |= FAILSAFE;
  }
  else
  {
    // we got a valid RC measurement for all channels and pwm is active
    // Clear the FAILSAFE bit
    _armed_state &= ~(FAILSAFE);
  }

  return failsafe;
}


bool check_mode()
{
  static uint32_t prev_time_ms = 0;
  static uint32_t time_sticks_have_been_in_arming_position_ms = 0;

  uint32_t now_ms = clock_millis();

  // see it has been at least 20 ms
  uint32_t dt = now_ms-prev_time_ms;
  if (dt < 20)
  {
    return false;
  }

  // if it has, then do stuff
  prev_time_ms = now_ms;

  // check for failsafe mode
  if (check_failsafe())
  {
    return true;
  }
  else
  {
    // check for arming switch
    if (!rc_switch_mapped(RC_SWITCH_ARM))
    {
      if (!(_armed_state & ARMED))
      {
        // if left stick is down and to the right
        if (rc_stick(RC_STICK_F) < get_param_float(PARAM_ARM_THRESHOLD)
            && rc_stick(RC_STICK_Z) > (1.0f - get_param_float(PARAM_ARM_THRESHOLD)))
        {
          time_sticks_have_been_in_arming_position_ms += dt;
        }
        else
        {
          time_sticks_have_been_in_arming_position_ms = 0;
        }
        if (time_sticks_have_been_in_arming_position_ms > 500)
        {
          if (arm())
            time_sticks_have_been_in_arming_position_ms = 0;
        }
      }
      else // _armed_state is ARMED
      {
        // if left stick is down and to the left
        if (rc_stick(RC_STICK_F) < get_param_float(PARAM_ARM_THRESHOLD)
            && rc_stick(RC_STICK_Z) < -(1.0f - get_param_float(PARAM_ARM_THRESHOLD)))
        {
          time_sticks_have_been_in_arming_position_ms += dt;
        }
        else
        {
          time_sticks_have_been_in_arming_position_ms = 0;
        }
        if (time_sticks_have_been_in_arming_position_ms > 500)
        {
          disarm();
          time_sticks_have_been_in_arming_position_ms = 0;
        }
      }
    }
    else
    {
      if (rc_switch(RC_SWITCH_ARM))
      {
        if ( !(_armed_state & ARMED))
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
