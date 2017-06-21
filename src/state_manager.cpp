#include "state_manager.h"
#include "rosflight.h"

namespace rosflight_firmware
{

StateManager::StateManager(ROSflight& parent) :
  RF_(parent)
{}

void StateManager::init()
{
  update_fsm(EVENT_INITIALIZED);
  process_errors();

  // Initialize LEDs
  RF_.board_->led1_off();
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
}

void StateManager::clear_error(uint16_t error)
{
  // Clear the error code
  state_.error_codes &= ~(error);

  // If there are no errors, tell the FSM
  process_errors();
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
    case EVENT_ERROR:
    case EVENT_RC_LOST:
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
        fsm_state_ = FSM_STATE_ARMED;
      }
      break;
    }
    break;

  case FSM_STATE_ERROR:
    switch (event)
    {
    case EVENT_NO_ERROR:
      state_.error = false;
      fsm_state_ = FSM_STATE_PREFLIGHT;
      break;
    }

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
    }

  case FSM_STATE_ARMED:
    switch (event)
    {
    case EVENT_RC_LOST:
      state_.failsafe = true;
      fsm_state_ = FSM_STATE_FAILSAFE;
      break;
    case EVENT_REQUEST_DISARM:
      state_.armed = false;
      fsm_state_ = FSM_STATE_PREFLIGHT;
      break;
    case EVENT_ERROR:
      state_.error = true;
      break;
    }

  case FSM_STATE_FAILSAFE:
    switch (event)
    {
    case EVENT_RC_FOUND:
      state_.failsafe = false;
      fsm_state_ = FSM_STATE_ARMED;
      break;
    }
  }
}

void StateManager::process_errors()
{
  if (state_.error_codes & ERROR_RC_LOST)
    set_event(EVENT_RC_LOST);
  else if (state_.error_codes)
    set_event(EVENT_ERROR);
  else
    set_event(EVENT_NO_ERROR);
}

void StateManager::update_leds()
{
  // off if disarmed, on if armed
  if (!state_.armed)
    RF_.board_->led1_off();
  else
    RF_.board_->led1_on();

  if (state_.error)
  {
    if (led_blink_counter_++ > 25)
    {
      RF_.board_->led1_toggle();
      led_blink_counter_ = 0;
    }
  }

  if (state_.failsafe)
  {
    if (led_blink_counter_++ > 13)
    {
      RF_.board_->led1_toggle();
      led_blink_counter_ = 0;
    }
  }
}

} //namespace rosflight_firmware
