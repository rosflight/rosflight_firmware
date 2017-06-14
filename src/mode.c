/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
error_state_t _error_state;


void init_mode(void)
{
  _armed_state = 0x00;
  _error_state = ERROR_NONE;
}

bool arm(void)
{
  if (_error_state)
  {
    mavlink_log_error("Unable to arm due to error code %d", _error_state);
    return false;
  }

  else if (get_param_int(PARAM_CALIBRATE_GYRO_ON_ARM))
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

  else
  {
    if (!(_armed_state & ARMED))
    {
      _armed_state |= ARMED;
      led1_on();
      return true;
    }
    return false;
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

    // Set the RC Lost error flag
    _error_state |= ERROR_RC_LOST;
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

    // clear the RC Lost error
    _error_state &= ~(ERROR_RC_LOST);
    if (_armed_state & ARMED)
      led1_on();
    else
      led1_off();
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
