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
#include <stdlib.h>

#include "board.h"
#include "param.h"
#include "rc.h"
#include "mux.h"
#include "mode.h"

#include "mavlink_util.h"
#include "mavlink_log.h"

typedef struct
{
  uint8_t channel;
  bool one_sided;
} rc_stick_config_t;

typedef struct
{
  uint8_t channel;
  int8_t direction;
  bool mapped;
} rc_switch_config_t;

static rc_stick_config_t sticks[RC_STICKS_COUNT];
static rc_switch_config_t switches[RC_SWITCHES_COUNT];

static float stick_values[RC_STICKS_COUNT];
static bool switch_values[RC_SWITCHES_COUNT];

void init_sticks(void)
{
  sticks[RC_STICK_X].channel = get_param_int(PARAM_RC_X_CHANNEL);
  sticks[RC_STICK_X].one_sided = false;

  sticks[RC_STICK_Y].channel = get_param_int(PARAM_RC_Y_CHANNEL);
  sticks[RC_STICK_Y].one_sided = false;

  sticks[RC_STICK_Z].channel = get_param_int(PARAM_RC_Z_CHANNEL);
  sticks[RC_STICK_Z].one_sided = false;

  sticks[RC_STICK_F].channel = get_param_int(PARAM_RC_F_CHANNEL);
  sticks[RC_STICK_F].one_sided = true;
}

static void init_switches()
{
  switches[RC_SWITCH_ARM].channel               = get_param_int(PARAM_RC_ARM_CHANNEL);
  switches[RC_SWITCH_ATT_OVERRIDE].channel      = get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_THROTTLE_OVERRIDE].channel = get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
  switches[RC_SWITCH_ATT_TYPE].channel          = get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);

  for (rc_switch_t chan = RC_SWITCH_ARM; chan < RC_SWITCHES_COUNT; chan++)
  {
    //check for a valid stick mapping. Channels 0-3 are reserved for FXYZ
    switches[chan].mapped = switches[chan].channel > 3 && switches[chan].channel < get_param_int(PARAM_RC_NUM_CHANNELS);
    if (!switches[chan].mapped)
    {
      mavlink_log_error("invalid RC switch channel assignment: %d", switches[chan].channel); // TODO use parameter name
    }

    //get switch toggle direction from associated param
    switches[chan].direction = 1;
    switch (switches[chan].channel)
    {
    case 4:
      switches[chan].direction = get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
      break;
    case 5:
      switches[chan].direction = get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
      break;
    case 6:
      switches[chan].direction = get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
      break;
    case 7:
      switches[chan].direction = get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
      break;
    }
  }
}

void init_rc()
{
  init_sticks();
  init_switches();
}

float rc_stick(rc_stick_t channel)
{
  return stick_values[channel];
}

bool rc_switch(rc_switch_t channel)
{
  return switch_values[channel];
}

bool rc_switch_mapped(rc_switch_t channel)
{
  return switches[channel].mapped;
}

bool receive_rc()
{
  static uint32_t last_rc_receive_time = 0;

  uint32_t now = clock_millis();

  // if it has been more than 20ms then look for new RC values and parse them
  if (now - last_rc_receive_time < 20)
  {
    return false;
  }
  last_rc_receive_time = now;

  // read and normalize stick values
  for (rc_stick_t channel = 0; channel < RC_STICKS_COUNT; channel++)
  {
    uint16_t pwm = pwm_read(sticks[channel].channel);
    if (sticks[channel].one_sided) //generally only F is one_sided
    {
      stick_values[channel] = (float)(pwm - 1000) / (1000.0);
    }
    else
    {
      stick_values[channel] = (float)(2*(pwm - 1500) / (1000.0));
    }
  }

  // read and interpret switch values
  for (rc_switch_t channel = 0; channel < RC_SWITCHES_COUNT; channel++)
  {
    if (switches[channel].mapped)
    {
      //switch is on/off dependent on its default direction as set in the params/init_switches
      if (switches[channel].direction <  0)
      {
        switch_values[channel] = pwm_read(switches[channel].channel) < 1500;
      }
      else
      {
        switch_values[channel] = pwm_read(switches[channel].channel) >= 1500;
      }
    }
    else
    {
      switch_values[channel] = false;
    }
  }

  // Signal to the mux that we need to compute a new combined command
  _new_command = true;
  return true;
}
