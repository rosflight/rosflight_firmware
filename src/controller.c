#include <stdint.h>
#include <stdbool.h>

#include <turbotrig/turbotrig.h>
#include <breezystm32/breezystm32.h>

#include "controller.h"
#include "param.h"
#include "mux.h"
#include "estimator.h"


int32_t sat(int32_t value, int32_t max)
{
  if (abs(value) > abs(max))
  {
    value = max*sign(value);
  }
  return value;
}


control_t altitude_controller(control_t altitude_command){}
control_t attitude_controller(control_t attitude_command)
{

}


control_t rate_controller(control_t rate_command, uint32_t now)
{
  control_t motor_command;

  static int32_t z_integrator = 0;
  static int32_t prev_time = 0;

  int32_t dt = (int32_t) now - prev_time;
  prev_time = now;


  // Set values
  if (rate_command.x.active && rate_command.x.type == RATE)
  {
    int32_t error = rate_command.x.value - _current_state.p/1000;
    motor_command.x.value = sat((error *_params.values[PARAM_PID_ROLL_RATE_P])/1000, _params.values[PARAM_MAX_COMMAND]);
    motor_command.x.type = PASSTHROUGH;
  }
  else
  {
    motor_command.x = rate_command.x;
  }

  if (rate_command.y.active && rate_command.y.type == RATE)
  {
    int32_t error = rate_command.y.value - _current_state.q/1000;
    motor_command.y.value = sat((error*_params.values[PARAM_PID_PITCH_RATE_P])/1000, _params.values[PARAM_MAX_COMMAND]);
    motor_command.y.type = PASSTHROUGH;
  }
  else
  {
    motor_command.y = rate_command.y;
  }

  if (rate_command.z.active && rate_command.z.type == RATE)
  {
    int32_t error = rate_command.z.value - _current_state.r/1000;
    z_integrator += error * dt;
    // integrator anti-windup - TEST THIS
    motor_command.z.value = (error*_params.values[PARAM_PID_YAW_RATE_P])/1000 + (z_integrator*_params.values[PARAM_PID_YAW_RATE_I])/1000000;
    if ( abs(motor_command.z.value) > _params.values[PARAM_MAX_COMMAND])
    {
      int32_t integrator_overshoot = motor_command.z.value - _params.values[PARAM_MAX_COMMAND]*sign(motor_command.z.value);
      motor_command.z.value -= integrator_overshoot;
      z_integrator -= (1000000*integrator_overshoot)/_params.values[PARAM_PID_YAW_RATE_I];
    }
    motor_command.z.type = PASSTHROUGH;
  }
  else
  {
    motor_command.z = rate_command.z;
  }
}

void anti_windup(int32_t error, int32_t integrator, int16_t kp, int16_t ki)
{

}
