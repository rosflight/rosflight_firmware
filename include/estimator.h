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
  vector_t euler;
  vector_t omega;

//  float p;
//  float q;
//  float r;

//  float phi;
//  float theta;
//  float psi;

  float altitude;

} state_t;

extern state_t _current_state;

void init_estimator(bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer);
void run_estimator();
#ifdef __cplusplus
}
#endif
