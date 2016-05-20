typedef enum controlMode_t{
  ALT_MODE,
  ATTITUDE_MODE,
  RATE_MODE,
  PASSTHROUGH
}controlMode_t;
controlMode_t onboard_control_mode;
controlMode_t rc_control_mode;
controlMode_t composite_control_mode;

typedef enum armedState_t{
  ARMED,
  DISARMED
}armedState_t;
armedState_t armed_state;

typedef enum overrideMode_t{
  OFFBOARD,
  OFFBOARD_MIN_THROTTLE,
  MANUAL_RC
}overrideMode_t;
overrideMode_t override_mode;

typedef enum errorState_t{
  INVALID_CONTROL_MODE,
  INVALID_ARMED_STATE,
}errorState_t;
errorState_t error_state;
