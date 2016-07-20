#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <turbotrig/turbotrig.h>
#include <breezystm32/breezystm32.h>

#include "param.h"
#include "mixer.h"
#include "mux.h"
#include "estimator.h"

#include "controller.h"
#include <stdio.h>

int fsign(float y)
{
  return (0 < y) - (y < 0);
}

float fsat(float value, float max)
{
  if (abs(value) > abs(max))
  {
    value = max*sign(value);
  }
  return value;
}

int32_t sat(int32_t value, int32_t max)
{
  if (abs(value) > abs(max))
  {
    value = max*sign(value);
  }
  return value;
}

void run_controller(uint32_t now)
{
  control_t rate_command = rate_controller(_combined_control, now);
  control_t outgoing_command = attitude_controller(rate_command, now);
  _command.x = (int32_t)outgoing_command.x.value;
  _command.y = (int32_t)outgoing_command.y.value;
  _command.z = (int32_t)outgoing_command.z.value;
  _command.F = (int32_t)outgoing_command.F.value;
}


control_t rate_controller(control_t rate_command, uint32_t now)
{
  static int32_t prev_time = 0;

  control_t motor_command = rate_command;

  float dt = (float)(now - prev_time)*1e-6;
  prev_time = now;

  // Set values
  if (rate_command.x.active && rate_command.x.type == RATE)
  {
    static float integrator = 0.0;
    float error = rate_command.x.value - _current_state.p;
    if(get_param_float(PARAM_PID_ROLL_RATE_I) > 0)
      integrator += error*dt;
    motor_command.x.value = fsat(error * get_param_float(PARAM_PID_ROLL_RATE_P)
                                 + integrator * get_param_float(PARAM_PID_ROLL_RATE_I),
                                 _params.values[PARAM_MAX_COMMAND]);
    motor_command.x.type = PASSTHROUGH;
  }

  if (rate_command.y.active && rate_command.y.type == RATE)
  {
    static float integrator = 0.0;
    float error = rate_command.y.value - _current_state.q;
    if(get_param_float(PARAM_PID_PITCH_RATE_I) > 0)
      integrator += error*dt;
    motor_command.y.value = fsat(error * get_param_float(PARAM_PID_PITCH_RATE_P)
                                 + integrator * get_param_float(PARAM_PID_PITCH_RATE_I),
                                 _params.values[PARAM_MAX_COMMAND]);
    motor_command.y.type = PASSTHROUGH;
  }

  if (rate_command.z.active && rate_command.z.type == RATE)
  {
    float error = rate_command.z.value - _current_state.r;
    motor_command.z.value = error * get_param_float(PARAM_PID_YAW_RATE_P);
    motor_command.z.type = PASSTHROUGH;
    /// TODO Add integrator to controller (with anti-windup)
  }
  return motor_command;
}

control_t attitude_controller(control_t attitude_command, uint32_t now)
{
  static int32_t prev_time = 0;

  control_t rate_command = attitude_command;

  float dt = (float)(now - prev_time)*1e-6;
  prev_time = now;

  if (attitude_command.x.type == ANGLE)
  {
    static float integrator = 0.0f;
    float error = attitude_command.x.value - _current_state.phi;
    integrator += error*dt;
    rate_command.x.value = error*get_param_float(PARAM_PID_ROLL_ANGLE_P)
                           + integrator*get_param_float(PARAM_PID_ROLL_ANGLE_I)
                           - _current_state.p*get_param_float(PARAM_PID_ROLL_ANGLE_D);
    // integrator anti-windup <-- Check this before fielding
//    if (abs(rate_command.x.value) > _params.values[PARAM_MAX_ROLL_RATE])
//    {
//      // find the remaining space after the P and D terms to saturate
//      int32_t space = _params.values[PARAM_MAX_ROLL_RATE]
//                      - (error*_params.values[PARAM_PID_ROLL_ANGLE_P])/1000
//                      + (_current_state.p/1000 *_params.values[PARAM_PID_ROLL_ANGLE_D]);;
//      // Make the integrator fill that space
//      x_integrator = (space*1000)/_params.values[PARAM_PID_ROLL_ANGLE_I];
//      // Make sure the integrator never goes negative
//      x_integrator = (x_integrator > 0) ? x_integrator : 0;
//      // Saturate the signal
//      rate_command.x.value = sat(rate_command.x.value, (int32_t)_params.values[PARAM_MAX_ROLL_RATE]);
//    }
    rate_command.x.type = PASSTHROUGH;
  }

  if (attitude_command.y.type == ANGLE)
  {
    static float integrator = 0.0f;
    float error = attitude_command.y.value - _current_state.theta;
    integrator += error*dt;
    rate_command.y.value = error * get_param_float(PARAM_PID_PITCH_ANGLE_P);
                           + integrator * get_param_float(PARAM_PID_PITCH_ANGLE_I)
                           - _current_state.q * get_param_float(PARAM_PID_PITCH_ANGLE_D);
    // integrator anti-windup
//    if (abs(rate_command.y.value) > _params.values[PARAM_MAX_PITCH_RATE])
//    {
//      int32_t space = _params.values[PARAM_MAX_PITCH_RATE]
//                      - (error*_params.values[PARAM_PID_PITCH_ANGLE_P])/1000
//                      + (_current_state.p/1000 *_params.values[PARAM_PID_PITCH_ANGLE_D]);;
//      y_integrator = (space*1000)/_params.values[PARAM_PID_PITCH_ANGLE_I];
//      y_integrator = (y_integrator > 0) ? y_integrator : 0;
//      rate_command.y.value = sat(rate_command.y.value, (int32_t)_params.values[PARAM_MAX_PITCH_RATE]);
//    }
    rate_command.y.type = PASSTHROUGH;
  }

  return rate_command;
}


//control_t altitude_controller(control_t altitude_command)
//{
//  // right now, this doesn't do anything, because I don't have an estimate of altitude
//  control_t attitude_command = altitude_command;
//  attitude_command.F.type = THROTTLE;
//  return attitude_command;
//}


#ifdef __cplusplus
}
#endif

