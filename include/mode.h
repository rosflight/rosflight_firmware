enum controlMode_t{
  ALT_MODE,
  ATTITUDE_MODE,
  RATE_MODE,
  PASSTHROUGH
};
controlMode_t onboard_control_mode;
controlMode_t rc_control_mode;
controlMode_t composite_control_mode;

enum armedState_t{
  ARMED,
  DISARMED
};
armedState_t armed_state;

enum overriedMode_t{
  OFFBOARD,
  OFFBOARD_MIN_THROTTLE,
  MANUAL_RC
};
overriedMode_t override_mode;

enum errorState_t{
  INVALID_CONTROL_MODE,
  INVALID_ARMED_STATE,
};
errorState_t error_state;
