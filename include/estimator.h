#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <turbotrig/turbovec.h>

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  float p;
  float q;
  float r;
  float phi;
  float theta;
  float psi;
  float altitude;
} state_t;

extern state_t _current_state;
extern vector_t _adaptive_gyro_bias;

void init_estimator(bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
void run_estimator(uint32_t now);
#ifdef __cplusplus
}
#endif
