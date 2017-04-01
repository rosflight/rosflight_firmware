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

#include "rc.h"

namespace rosflight
{

void RC::init_rc(Board *_board, Params *_params)
{
  params = _params;
  board = _board;
  init_sticks();
  init_switches();
}

float RC::rc_stick(rc_stick_t channel)
{
  new_command_ = false;
  return stick_values[channel];
}

bool RC::rc_switch(int16_t channel)
{
  return switch_values[channel];
}

bool RC::rc_switch_mapped(rc_switch_t channel)
{
  return switches[channel].mapped;
}


void RC::init_sticks(void)
{
  sticks[RC_STICK_X].channel = params->get_param_int(PARAM_RC_X_CHANNEL);
  sticks[RC_STICK_X].one_sided = false;

  sticks[RC_STICK_Y].channel = params->get_param_int(PARAM_RC_Y_CHANNEL);
  sticks[RC_STICK_Y].one_sided = false;

  sticks[RC_STICK_Z].channel = params->get_param_int(PARAM_RC_Z_CHANNEL);
  sticks[RC_STICK_Z].one_sided = false;

  sticks[RC_STICK_F].channel = params->get_param_int(PARAM_RC_F_CHANNEL);
  sticks[RC_STICK_F].one_sided = true;
}

void RC::init_switches()
{
  // Make sure that parameters for switch channels are correct
//  switches[RC_SWITCH_ARM].channel               = params->get_param_int(PARAM_RC_ARM_CHANNEL);
  switches[RC_SWITCH_ATT_OVERRIDE].channel      = params->get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_THROTTLE_OVERRIDE].channel = params->get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_ATT_TYPE].channel          = params->get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);

  for (uint8_t chan = 0; chan < (uint8_t)RC_SWITCHES_COUNT; chan++)
  {
    switches[chan].mapped = switches[chan].channel > 3 && switches[chan].channel < params->get_param_int(PARAM_RC_NUM_CHANNELS);
    if (!switches[chan].mapped)
    {
//      mavlink_log_error("invalid RC switch channel assignment: %d", switches[chan].channel); // TODO use parameter name
    }

    switches[chan].direction = 1;
    switch (chan)
    {
    case 4:
      switches[chan].direction = params->get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
      break;
    case 5:
      switches[chan].direction = params->get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
      break;
    case 6:
      switches[chan].direction = params->get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
      break;
    case 7:
      switches[chan].direction = params->get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
      break;
    }
  }
}


bool RC::receive_rc()
{
    static uint32_t last_rc_receive_time = 0;

    uint32_t now = board->clock_millis();

    // if it has been more than 20ms then look for new RC values and parse them
    if (now - last_rc_receive_time < 20)
    {
        return false;
    }
    last_rc_receive_time = now;

    // read and normalize stick values
    for (uint8_t channel = 0; channel < (uint8_t)RC_STICKS_COUNT; channel++)
    {
      uint16_t pwm = board->pwm_read(sticks[channel].channel);
      stick_values[channel] = (sticks[channel].one_sided ? 1.0f : 2.0f) * (float)(pwm - 1500)/1000.0;
    }

    // read and interpret switch values
    for (uint8_t channel = 0; channel < (uint8_t)RC_SWITCHES_COUNT; channel++)
    {
      if (switches[channel].mapped)
      {
        if (switches[channel].direction < 0)
        {
          switch_values[channel] = board->pwm_read(switches[channel].channel) < 1500;
        }
        else
        {
          switch_values[channel] = board->pwm_read(switches[channel].channel) >= 1500;
        }
      }
      else
      {
        switch_values[channel] = false;
      }
    }

    // Signal to the mux that we need to compute a new combined command
    new_command_ = true;
    return true;
}

bool RC::new_command()
{
  return new_command_;
}

}
