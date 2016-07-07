#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  int32_t p;
  int32_t q;
  int32_t r;
  int32_t phi;
  int32_t theta;
  int32_t psi;
} state_t;

extern int16_t _adaptive_gyro_bias[3];

extern state_t _current_state;

void init_estimator(bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
void run_estimator(uint32_t now);
