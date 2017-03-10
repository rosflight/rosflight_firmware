#include <stdint.h>

#include "mixer.h"

namespace rosflight {

void Mixer::init(Board *_board, Mux *_mux, Params *_params, Arming_FSM *_fsm)
{
  board = _board;
  mux = _mux;
  params = _params;
  fsm = _fsm;
}


void Mixer::init_mixing()
{
  // We need a better way to choosing the mixer
  mixer_to_use = array_of_mixers[params->get_param_int(PARAM_MIXER)];

  for (int8_t i=0; i<8; i++)
  {
    _outputs[i] = 0.0f;
    prescaled_outputs[i] = 0.0f;
  }
  _command.F = 0;
  _command.x = 0;
  _command.y = 0;
  _command.z = 0;
}

void Mixer::init_PWM()
{
  bool useCPPM = false;
  if (params->get_param_int(PARAM_RC_TYPE) == 1)
  {
    useCPPM = true;
  }
  int16_t motor_refresh_rate = params->get_param_int(PARAM_MOTOR_PWM_SEND_RATE);
  int16_t off_pwm = params->get_param_int(PARAM_MOTOR_MIN_PWM);
  board->pwm_init(useCPPM, motor_refresh_rate, off_pwm);
}


void Mixer::write_motor(uint8_t index, float value)
{
  if (fsm->_armed_state == ARMED)
  {
    if (value > 1.0)
    {
      value = 1.0;
    }
    else if (value < params->get_param_float(PARAM_MOTOR_IDLE_THROTTLE) && params->get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED))
    {
      value = params->get_param_int(PARAM_MOTOR_IDLE_THROTTLE);
    }
    else if (value < 0.0)
    {
      value = 0.0;
    }
  }
  else
  {
    value = 0.0;
  }
  _outputs[index] = value;
  int32_t pwm_us = value * (params->get_param_int(PARAM_MOTOR_MAX_PWM) - params->get_param_int(PARAM_MOTOR_MIN_PWM)) + params->get_param_int(PARAM_MOTOR_MIN_PWM);
  board->pwm_write(index, pwm_us);
}


void Mixer::write_servo(uint8_t index, float value)
{
  if (value > 1.0)
  {
    value = 1.0;
  }
  else if (value < -1.0)
  {
    value = -1.0;
  }
  _outputs[index] = value;
  board->pwm_write(index, _outputs[index] * 1000 + 1500);
}


void Mixer::mix_output()
{
  float max_output = 1.0f;

  // Reverse Fixedwing channels just before mixing if we need to
  if (get_param_int(PARAM_FIXED_WING))
  {
    _command.x *= get_param_int(PARAM_AILERON_REVERSE) ? -1 : 1;
    _command.y *= get_param_int(PARAM_ELEVATOR_REVERSE) ? -1 : 1;
    _command.z *= get_param_int(PARAM_RUDDER_REVERSE) ? -1 : 1;
  }

  for (int8_t i=0; i<8; i++)
  {
    if (mixer_to_use->output_type[i] != NONE)
    {
      // Matrix multiply to mix outputs
      prescaled_outputs[i] = (_command.F*mixer_to_use->F[i] + _command.x*mixer_to_use->x[i] +
                              _command.y*mixer_to_use->y[i] + _command.z*mixer_to_use->z[i]);

      // Save off the largest control output if it is greater than 1.0 for future scaling
      if (prescaled_outputs[i] > max_output)
      {
        max_output = prescaled_outputs[i];
      }
    }
  }

  // saturate outputs to maintain controllability even during aggressive maneuvers
  float scale_factor = 1.0;
  if (max_output > 1.0)
  {
    scale_factor = 1.0/max_output;
  }

  // Reverse Fixedwing channels
  if (params->get_param_int(PARAM_FIXED_WING))
  {
    prescaled_outputs[0] *= params->get_param_int(PARAM_AILERON_REVERSE) ? -1 : 1;
    prescaled_outputs[1] *= params->get_param_int(PARAM_ELEVATOR_REVERSE) ? -1 : 1;
    prescaled_outputs[3] *= params->get_param_int(PARAM_RUDDER_REVERSE) ? -1 : 1;
  }

  // Add in GPIO inputs from Onboard Computer
  for (int8_t i=0; i<8; i++)
  {
    output_type_t output_type = mixer_to_use->output_type[i];
    if (mixer_to_use->output_type[i] == M)
    {
      prescaled_outputs[i] = (prescaled_outputs[i])*scale_factor; // scale all outputs by scale factor
    }

    if (output_type == NONE)
    {
      // Incorporate GPIO on not already reserved outputs
      prescaled_outputs[i] = _GPIO_outputs[i];
      output_type = _GPIO_output_type[i];
    }

    // Write output to motors
    if (mixer_to_use->output_type[i] == S)
    {
      write_servo(i, prescaled_outputs[i]);
    }
    else if (mixer_to_use->output_type[i] == M)
    {
      // scale all motor outputs by scale factor (this is usually 1.0, unless we saturated)
      prescaled_outputs[i] *= scale_factor;
      write_motor(i, prescaled_outputs[i]);
    }
  }
}

}
