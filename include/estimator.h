#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  int16_t p;
  int16_t q;
  int16_t r;

  int32_t phi;
  int32_t theta;
  int32_t psi;
} state_t;

extern state_t _current_state;

void init_estimator();
void run_estimator(int32_t dt);
