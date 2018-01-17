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

#include "state_manager.h"
#include "rosflight.h"

namespace rosflight_firmware
{

StateManager::StateManager(ROSflight& parent) :
  RF_(parent), fsm_state_(FSM_STATE_INIT)
{
  state_.armed = false;
  state_.error = false;
  state_.failsafe = false;
  state_.error_codes = 0x00;
}

void StateManager::init()
{
  set_event(EVENT_INITIALIZED);
  process_errors();

  // Initialize LEDs
  RF_.board_.led1_off();
}

void StateManager::run()
{
  // I'm putting this here for the case where we've switched states with existing errors,
  // but those errors might not have been processed yet for the new state. We could replace
  // this with a recursive call to process_errors() if the state changed in update_fsm()?
  process_errors(); // check for any error events
  update_leds();
}

void StateManager::set_error(uint16_t error)
{
  // Set the error code
  state_.error_codes |= error;

  // Tell the FSM that we have had an error change
  process_errors();
  RF_.comm_manager_.update_status();
}

void StateManager::clear_error(uint16_t error)
{
  // If this error code was set,
  if (state_.error_codes & error)
  {
    // Clear the error code
    state_.error_codes &= ~(error);

    // If there are no errors, tell the FSM
    process_errors();

    // Send a status update (for logging)
    RF_.comm_manager_.update_status();
  }
}

void StateManager::set_event(StateManager::Event event)
{
  switch (fsm_state_)
  {
  case FSM_STATE_INIT:
    if (event == EVENT_INITIALIZED)
    {
      fsm_state_ = FSM_STATE_PREFLIGHT;
    }
    break;

  case FSM_STATE_PREFLIGHT:
    switch (event)
    {
    case EVENT_RC_FOUND:
      clear_error(ERROR_RC_LOST);
      state_.failsafe = false;
      break;
    case EVENT_RC_LOST:
      set_error(ERROR_RC_LOST);
      break;
    case EVENT_ERROR:
      state_.error = true;
      fsm_state_ = FSM_STATE_ERROR;
      break;
    case EVENT_REQUEST_ARM:
      if (RF_.params_.get_param_int(PARAM_CALIBRATE_GYRO_ON_ARM))
      {
        fsm_state_ = FSM_STATE_CALIBRATING;
        RF_.sensors_.start_gyro_calibration();
      }
      else
      {
        state_.armed = true;
        RF_.comm_manager_.update_status();
        fsm_state_ = FSM_STATE_ARMED;
      }
      break;
    default:
      break;
    }
    break;

  case FSM_STATE_ERROR:
    switch (event)
    {
    case EVENT_RC_LOST:
      set_error(ERROR_RC_LOST); // sometimes redundant, but reports RC lost error if another error got reported first
      break;
    case EVENT_RC_FOUND:
      clear_error(ERROR_RC_LOST);
      state_.failsafe = false;
      break;
    case EVENT_NO_ERROR:
      state_.error = false;
      fsm_state_ = FSM_STATE_PREFLIGHT;
      break;
    case EVENT_REQUEST_ARM:
      RF_.comm_manager_.log(CommLink::LogSeverity::LOG_ERROR, "unable to arm due to error code 0x%x", state_.error_codes);
      break;
    default:
      break;
    }
    break;

  case FSM_STATE_CALIBRATING:
    switch (event)
    {
    case EVENT_CALIBRATION_COMPLETE:
      state_.armed = true;
      fsm_state_ = FSM_STATE_ARMED;
      break;
    case EVENT_CALIBRATION_FAILED:
      fsm_state_ = FSM_STATE_PREFLIGHT;
      break;
    case EVENT_RC_LOST:
      set_error(ERROR_RC_LOST);
      break;
    case EVENT_ERROR:
      fsm_state_ = FSM_STATE_ERROR;
      state_.error = true;
      break;
    case EVENT_NO_ERROR:
      state_.error = false;
      break;
    default:
      break;
    }
    break;

  case FSM_STATE_ARMED:
    switch (event)
    {
    case EVENT_RC_LOST:
      state_.failsafe = true;
      RF_.comm_manager_.update_status();
      fsm_state_ = FSM_STATE_FAILSAFE;
      set_error(ERROR_RC_LOST);
      break;
    case EVENT_REQUEST_DISARM:
      state_.armed = false;
      RF_.comm_manager_.update_status();
      if (state_.error)
        fsm_state_ = FSM_STATE_ERROR;
      else
        fsm_state_ = FSM_STATE_PREFLIGHT;
      break;
    case EVENT_ERROR:
      state_.error = true;
      break;
    case EVENT_NO_ERROR:
      state_.error = false;
      break;
    default:
      break;
    }
    break;

  case FSM_STATE_FAILSAFE:
    switch (event)
    {
    case EVENT_ERROR:
      state_.error = true;
      break;
    case EVENT_REQUEST_DISARM:
      state_.armed = false;
      fsm_state_ = FSM_STATE_ERROR;
      break;
    case EVENT_RC_FOUND:
      RF_.comm_manager_.update_status();
      state_.failsafe = false;
      fsm_state_ = FSM_STATE_ARMED;
      clear_error(ERROR_RC_LOST);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

void StateManager::process_errors()
{
  if (state_.error_codes)
    set_event(EVENT_ERROR);
  else
    set_event(EVENT_NO_ERROR);
}

void StateManager::update_leds()
{
  // blink fast if in failsafe
  if (state_.failsafe)
  {
    if (next_led_blink_ms_ < RF_.board_.clock_millis())
    {
      RF_.board_.led1_toggle();
      next_led_blink_ms_ =  RF_.board_.clock_millis() + 100;
    }
  }
  // blink slowly if in error
  else if (state_.error)
  {
    if (next_led_blink_ms_ < RF_.board_.clock_millis())
    {
      RF_.board_.led1_toggle();
      next_led_blink_ms_ =  RF_.board_.clock_millis() + 500;
    }
  }
  // off if disarmed, on if armed
  else if (!state_.armed)
    RF_.board_.led1_off();
  else
    RF_.board_.led1_on();
}

} //namespace rosflight_firmware
