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

#include <cstring>

#include "rc.h"
#include "rosflight.h"

namespace rosflight_firmware
{

RC::RC(ROSflight &_rf) :
  RF_(_rf)
{}

void RC::init()
{
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_ARM_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_X_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_Y_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_Z_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_F_CHANNEL);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_SWITCH_5_DIRECTION);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_SWITCH_6_DIRECTION);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_SWITCH_7_DIRECTION);
  RF_.params_.add_callback(std::bind(&RC::param_change_callback, this, std::placeholders::_1), PARAM_RC_SWITCH_8_DIRECTION);
  init_rc();
  new_command_ = false;
}

void RC::init_rc()
{
  init_sticks();
  init_switches();
}

void RC::param_change_callback(uint16_t param_id)
{
  (void) param_id; // suppress unused parameter warning
  init_rc();
}

float RC::stick(Stick channel)
{
  return stick_values[channel];
}

bool RC::switch_on(Switch channel)
{
  return switch_values[channel];
}

bool RC::switch_mapped(Switch channel)
{
  return switches[channel].mapped;
}


void RC::init_sticks(void)
{
  sticks[STICK_X].channel = RF_.params_.get_param_int(PARAM_RC_X_CHANNEL);
  sticks[STICK_X].one_sided = false;

  sticks[STICK_Y].channel = RF_.params_.get_param_int(PARAM_RC_Y_CHANNEL);
  sticks[STICK_Y].one_sided = false;

  sticks[STICK_Z].channel = RF_.params_.get_param_int(PARAM_RC_Z_CHANNEL);
  sticks[STICK_Z].one_sided = false;

  sticks[STICK_F].channel = RF_.params_.get_param_int(PARAM_RC_F_CHANNEL);
  sticks[STICK_F].one_sided = true;
}

void RC::init_switches()
{
  for (uint8_t chan = 0; chan < static_cast<uint8_t>(SWITCHES_COUNT); chan++)
  {
    char channel_name[18];
    switch (chan)
    {
    case SWITCH_ARM:
      strcpy(channel_name, "ARM");
      switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_ARM_CHANNEL);
      break;
    case SWITCH_ATT_OVERRIDE:
      strcpy(channel_name, "ATTITUDE OVERRIDE");
      switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL);
      break;
    case SWITCH_THROTTLE_OVERRIDE:
      strcpy(channel_name, "THROTTLE OVERRIDE");
      switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL);
      break;
    case SWITCH_ATT_TYPE:
      strcpy(channel_name, "ATTITUDE TYPE");
      switches[chan].channel = RF_.params_.get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL);
      break;
    default:
      strcpy(channel_name, "INVALID");
      switches[chan].channel = 255;
      break;
    }

    switches[chan].mapped = switches[chan].channel > 3
                            && switches[chan].channel < RF_.params_.get_param_int(PARAM_RC_NUM_CHANNELS);

    switch (switches[chan].channel)
    {
    case 4:
      switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_5_DIRECTION);
      break;
    case 5:
      switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_6_DIRECTION);
      break;
    case 6:
      switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_7_DIRECTION);
      break;
    case 7:
      switches[chan].direction = RF_.params_.get_param_int(PARAM_RC_SWITCH_8_DIRECTION);
      break;
    default:
      switches[chan].direction = 1;
      break;
    }

    if (switches[chan].mapped)
      RF_.comm_manager_.log(CommLink::LogSeverity::LOG_INFO, "%s switch mapped to RC channel %d", channel_name, switches[chan].channel);
    else
      RF_.comm_manager_.log(CommLink::LogSeverity::LOG_INFO, "%s switch not mapped", channel_name);
  }
}

bool RC::check_rc_lost()
{
  bool failsafe = false;

  // If the board reports that we have lost RC, tell the state manager
  if (RF_.board_.pwm_lost())
  {
    failsafe = true;
  }
  else
  {
    // go into failsafe if we get an invalid RC command for any channel
    for (int8_t i = 0; i<RF_.params_.get_param_int(PARAM_RC_NUM_CHANNELS); i++)
    {
      if (RF_.board_.pwm_read(i) < 900 || RF_.board_.pwm_read(i) > 2100)
      {
        failsafe = true;
      }
    }
  }

  if (failsafe)
    // set the RC Lost error flag
    RF_.state_manager_.set_event(StateManager::EVENT_RC_LOST);
  else
    // Clear the RC Lost Error
    RF_.state_manager_.set_event(StateManager::EVENT_RC_FOUND);

  return failsafe;
}

void RC::look_for_arm_disarm_signal()
{
  uint32_t now_ms = RF_.board_.clock_millis();
  uint32_t dt = now_ms - prev_time_ms;
  prev_time_ms = now_ms;
  // check for arming switch
  if (!switch_mapped(SWITCH_ARM))
  {
    if (!RF_.state_manager_.state().armed) // we are DISARMED
    {
      // if left stick is down and to the right
      if ((RF_.rc_.stick(STICK_F) < RF_.params_.get_param_float(PARAM_ARM_THRESHOLD))
          && (RF_.rc_.stick(STICK_Z) > (1.0f - RF_.params_.get_param_float(PARAM_ARM_THRESHOLD))))
      {
        time_sticks_have_been_in_arming_position_ms += dt;
      }
      else
      {
        time_sticks_have_been_in_arming_position_ms = 0;
      }
      if (time_sticks_have_been_in_arming_position_ms > 1000)
      {
        RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);
      }
    }
    else // we are ARMED
    {
      // if left stick is down and to the left
      if (RF_.rc_.stick(STICK_F) < RF_.params_.get_param_float(PARAM_ARM_THRESHOLD)
          && RF_.rc_.stick(STICK_Z) < -(1.0f - RF_.params_.get_param_float(PARAM_ARM_THRESHOLD)))
      {
        time_sticks_have_been_in_arming_position_ms += dt;
      }
      else
      {
        time_sticks_have_been_in_arming_position_ms = 0;
      }
      if (time_sticks_have_been_in_arming_position_ms > 1000)
      {
        RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
        time_sticks_have_been_in_arming_position_ms = 0;
      }
    }
  }
  else // ARMING WITH SWITCH
  {
    if (RF_.rc_.switch_on(SWITCH_ARM))
    {
      if (!RF_.state_manager_.state().armed)
        RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_ARM);;
    }
    else
    {
      RF_.state_manager_.set_event(StateManager::EVENT_REQUEST_DISARM);
    }
  }
}


bool RC::run()
{
  uint32_t now = RF_.board_.clock_millis();

  // if it has been more than 20ms then look for new RC values and parse them
  if (now - last_rc_receive_time < 20)
  {
    return false;
  }
  last_rc_receive_time = now;


  // Check for rc lost
  if (check_rc_lost())
    return false;


  // read and normalize stick values
  for (uint8_t channel = 0; channel < static_cast<uint8_t>(STICKS_COUNT); channel++)
  {
    uint16_t pwm = RF_.board_.pwm_read(sticks[channel].channel);
    if (sticks[channel].one_sided) //generally only F is one_sided
    {
      stick_values[channel] = (static_cast<float>(pwm - 1000)) / 1000.0f;
    }
    else
    {
      stick_values[channel] = static_cast<float>(2*(pwm - 1500)) / (1000.0f);
    }
  }

  // read and interpret switch values
  for (uint8_t channel = 0; channel < static_cast<uint8_t>(SWITCHES_COUNT); channel++)
  {
    if (switches[channel].mapped)
    {
      if (switches[channel].direction < 0)
      {
        switch_values[channel] = RF_.board_.pwm_read(switches[channel].channel) < 1200;
      }
      else
      {
        switch_values[channel] = RF_.board_.pwm_read(switches[channel].channel) >= 1800;
      }
    }
    else
    {
      switch_values[channel] = false;
    }
  }

  // Look for arming and disarming signals
  look_for_arm_disarm_signal();

  // Signal to the mux that we need to compute a new combined command
  new_command_ = true;
  return true;
}

bool RC::new_command()
{
  if (new_command_)
  {
    new_command_ = false;
    return true;
  }
  else
    return false;;
}

}
