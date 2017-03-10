/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab, Provo UT
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

#include "arming_fsm.h"

namespace rosflight
{

Arming_FSM::Arming_FSM()
{
  _armed_state = DISARMED;
  prev_time_ms = 0;
  time_sticks_have_been_in_arming_position_ms = 0;
}

void Arming_FSM::init_mode(Board *_board, Sensors *_sensors, Params *_params, RC *_rc)
{
   rc_ = _rc;
   board_ = _board;
   sensors_ = _sensors;
   params_ = _params;
  _armed_state = DISARMED;
}

bool Arming_FSM::arm(void)
{
  started_gyro_calibration = false;
  if (!started_gyro_calibration && _armed_state & DISARMED)
  {
    sensors_->start_gyro_calibration();
    started_gyro_calibration = true;
    return false;
  }
  else if (sensors_->gyro_calibration_complete())
  {
    started_gyro_calibration = false;
    _armed_state = ARMED;
    board_->led1_on();
    return true;
  }
  return false;
}

void Arming_FSM::disarm(void)
{
  _armed_state = DISARMED;
  board_->led1_off();
}

bool Arming_FSM::check_failsafe(void)
{
  if (board_->pwm_lost())
  {
    // Set the FAILSAFE bit
    _armed_state = (ARMED) ? ARMED_FAILSAFE : DISARMED_FAILSAFE;
    return true;
  }

    // Set the RC Lost error flag
    _error_state |= ERROR_RC_LOST;
  }
  else
  {
    for (int8_t i = 0; i<params_->get_param_int(PARAM_RC_NUM_CHANNELS); i++)
    {
      if(board_->pwm_read(i) < 900 || board_->pwm_read(i) > 2100)
      {
        _armed_state = (ARMED) ? ARMED_FAILSAFE : DISARMED_FAILSAFE;

        // blink LED
        static uint8_t count = 0;
        if (count > 25)
        {
          board_->led1_toggle();
          count = 0;
        }
        count++;
        return true;
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
    _armed_state = (ARMED_FAILSAFE) ? ARMED : DISARMED;
    return false;
  }

  return failsafe;
}


bool Arming_FSM::check_mode()
{
  uint32_t now = board_->clock_millis();

  // see it has been at least 20 ms
  uint32_t dt = now-prev_time_ms;
  if (dt < 20)
  {
    return false;
  }

  // if it has, then do stuff
  prev_time_ms = now;

  // check for failsafe mode
  if (check_failsafe())
  {
    return true;
  }
  else
  {
    // check for arming switch
    if (params_->get_param_int(PARAM_ARM_STICKS))
    {
      if (!(_armed_state & ARMED))
      {
        // if left stick is down and to the right
        if (rc_->rc_low(RC::RC_F) && rc_->rc_high(RC::RC_Z))
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
        if (rc_->rc_low(RC::RC_F) && rc_->rc_low(RC::RC_Z))
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
      if (rc_->rc_switch(params_->get_param_int(PARAM_ARM_CHANNEL)))
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

}
