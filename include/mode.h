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
