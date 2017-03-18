#ifdef __cplusplus
extern "C" {
#endif

#pragma once

typedef enum
{
  ARMED = 0x01,
  FAILSAFE = 0x2,
} armed_state_t;
extern armed_state_t _armed_state;

typedef enum
{
  INVALID_CONTROL_MODE,
  INVALID_ARMED_STATE,
} error_state_t;
error_state_t _error_state;

void init_mode(void);
bool check_mode();

#ifdef __cplusplus
}
#endif

