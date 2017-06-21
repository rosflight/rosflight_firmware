#include "state_manager.h"
#include "rosflight.h"

namespace rosflight_firmware
{

StateManager::StateManager(ROSflight& parent) :
  rosflight_(parent)
{}

void StateManager::init()
{
  update_fsm(FSM_EVENT_INITIALIZED);
  process_errors();

  // TODO initialize LEDs
}

void StateManager::run()
{
  // I'm putting this here for the case where we've switched states with existing errors,
  // but those errors might not have been processed yet for the new state. We could replace
  // this with a recursive call to process_errors() if the state changed in update_fsm()?
  process_errors(); // check for any error events

  update_leds();
}

void StateManager::set_event(StateManager::Event event)
{
}

void StateManager::update_fsm(StateManager::FsmEvent event)
{
  switch (fsm_state_)
  {
  case FSM_STATE_INIT:
    if (event == FSM_EVENT_INITIALIZED)
    {
      fsm_state_ = FSM_STATE_PREFLIGHT;
    }
    break;

  case FSM_STATE_PREFLIGHT:
    switch (event)
    {
    case FSM_EVENT_ERROR:
    case FSM_EVENT_RC_LOST:
      state_.error = true;
      fsm_state_ = FSM_STATE_ERROR;
      break;
    case FSM_EVENT_REQUEST_ARM:
      state_.armed = true;
      fsm_state_ = FSM_STATE_ARMED;
      break;
    }
    break;

  case FSM_STATE_ARMED:
    switch (event)
    {
    case FSM_EVENT_RC_LOST:
      fsm_state_
    }
  }
}

void StateManager::process_errors()
{
  if (state_.error_codes & ERROR_RC_LOST)
    update_fsm(FSM_EVENT_RC_LOST);
  else if (state_.error_codes)
    update_fsm(FSM_EVENT_ERROR);
  else
    update_fsm(FSM_EVENT_NO_ERROR);
}

void StateManager::update_leds()
{
  // TODO off if disarmed
  // TODO on if armed
  // TODO slow blink if error
  // TODO fast blink if failsafe
}

} //namespace rosflight_firmware
