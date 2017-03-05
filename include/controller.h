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
  param_id_t kp_param_id;
  param_id_t ki_param_id;
  param_id_t kd_param_id;

  float *current_x;
  float *current_xdot;
  float *commanded_x;
  float *output;

  float max;
  float min;

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

void init_pid(pid_t *pid, param_id_t kp_param_id, param_id_t ki_param_id, param_id_t kd_param_id, float *current_x,
              float *current_xdot, float *commanded_x, float *output, float max, float min);
void run_pid(pid_t *pid, float dt);

void run_controller();
void init_controller();


#ifdef __cplusplus
}
#endif
