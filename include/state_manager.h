#ifndef ROSFLIGHT_FIRMWARE_STATE_MANAGER_H
#define ROSFLIGHT_FIRMWARE_STATE_MANAGER_H

#include <stdint.h>

namespace rosflight_firmware
{

class ROSflight;

class StateManager
{
  struct State
  {
    bool armed;
    bool failsafe;
    bool error;
    uint16_t error_codes;
  };

  enum Event
  {
    EVENT_INITIALIZED,
    EVENT_REQUEST_ARM,
    EVENT_REQUEST_DISARM,
    EVENT_RC_LOST,
    EVENT_RC_FOUND,
    EVENT_ERROR,
    EVENT_NO_ERROR,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_CALIBRATION_FAILED,
  };

  enum
  {
    ERROR_NONE = 0x0000,
    ERROR_INVALID_MIXER = 0x0001,
    ERROR_IMU_NOT_RESPONDING = 0x0002,
    ERROR_RC_LOST = 0x0004,
    ERROR_UNHEALTHY_ESTIMATOR = 0x0008,
    ERROR_TIME_GOING_BACKWARDS = 0x0010,
    ERROR_UNCALIBRATED_IMU = 0x0020,
  };

public:
  StateManager(ROSflight& parent);
  void init();
  void run();

  inline const State& state() const { return state_; }

  void set_event(Event event);
  void set_error(uint16_t error);
  void clear_error(uint16_t error);

private:
  ROSflight& RF_;
  State state_;

  int led_blink_counter_ = 0;

  enum FsmState
  {
    FSM_STATE_INIT,
    FSM_STATE_PREFLIGHT,
    FSM_STATE_ARMED,
    FSM_STATE_ERROR,
    FSM_STATE_FAILSAFE,
    FSM_STATE_CALIBRATING
  };

  enum FsmEvent
  {
    EVENT_INITIALIZED,
    EVENT_REQUEST_ARM,
    EVENT_REQUEST_DISARM,
    EVENT_RC_LOST,
    EVENT_RC_FOUND,
    EVENT_ERROR,
    EVENT_NO_ERROR,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_CALIBRATION_FAILED,
  };

  FsmState fsm_state_;
  void process_errors();

  void update_leds();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_STATE_MANAGER_H
