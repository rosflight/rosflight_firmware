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
  ERROR_NONE = 0x00,
  ERROR_INVALID_MIXER = 0x01,
  ERROR_IMU_NOT_RESPONDING = 0x02,
  ERROR_RC_LOST = 0x04,
  ERROR_UNHEALTHY_ESTIMATOR = 0x08,
} error_state_t;
extern error_state_t _error_state;

void init_mode(void);
bool check_mode();

#ifdef __cplusplus
}
#endif

