#ifndef ROSFLIGHT_FIRMWARE_STATE_MANAGER_H
#define ROSFLIGHT_FIRMWARE_STATE_MANAGER_H

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
    EVENT_GYRO_CALIBRATION_COMPLETE;
    EVENT_GYRO_CALIBRATION_FAILED;
  };

  enum Error
  {
    ERROR_MEH = 0x01;
  };

public:
  StateManager(ROSflight& parent);
  void init();
  void run();

  inline const State& state() const { return state_; }

  void set_event(Event event);
  void set_error(Error error);
  void clear_error(Error error);

private:
  ROSflight& rosflight_;
  State state_;

  enum FsmState;
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
    FSM_EVENT_INITIALIZED,
    FSM_EVENT_REQUEST_ARM,
    FSM_EVENT_REQUEST_DISARM,
    FSM_EVENT_RC_LOST,
    FSM_EVENT_RC_FOUND,
    FSM_EVENT_ERROR,
    FSM_EVENT_NO_ERROR
  };

  FsmState fsm_state_;
  void update_fsm(FsmEvent event);
  void process_errors();

  void update_leds();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_STATE_MANAGER_H
