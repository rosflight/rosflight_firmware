#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <turbotrig/turbovec.h>

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  quaternion_t q;
  vector_t omega;
  float roll;
  float pitch;
  float yaw;
  float altitude;
  uint64_t now_us;

} state_t;

extern state_t _current_state;

void reset_state();
void reset_adaptive_bias();
void init_estimator(bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
void run_estimator();
#ifdef __cplusplus
}
#endif
