#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "mux.h"
#include "param.h"

typedef struct
{
  float* kp;
  float* ki;
  float* kd;

  float* current_x;
  float* current_xdot;
  float* commanded_x;
  float* output;

  float* max;
  float* min;

  float integrator;
  float prev_time;
  float prev_x;
  float differentiator;
  float tau;
} pid_t;

pid_t pid_roll;
pid_t pid_roll_rate;
pid_t pid_pitch;
pid_t pid_pitch_rate;
pid_t pid_yaw_rate;
pid_t pid_altitude;

void init_pid(pid_t* pid, float* kp, float* ki, float* kd, float* current_x, float* current_xdot, float* commanded_x, float* output);
void run_pid(pid_t* pid);

void run_controller();
void init_controller();


#ifdef __cplusplus
}
#endif
