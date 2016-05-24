typedef enum
{
  ALT_MODE,
  ATTITUDE_MODE,
  RATE_MODE,
  PASSTHROUGH
} control_mode_t;
control_mode_t onboard_control_mode;
control_mode_t rc_control_mode;
control_mode_t composite_control_mode;

typedef enum
{
  ARMED,
  DISARMED
} armed_state_t;
armed_state_t armed_state;

typedef enum
{
  OFFBOARD,
  OFFBOARD_MIN_THROTTLE,
  MANUAL_RC
} override_mode_t;
override_mode_t override_mode;

typedef enum
{
  INVALID_CONTROL_MODE,
  INVALID_ARMED_STATE,
} error_state_t;
error_state_t error_state;
