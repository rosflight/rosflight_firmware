#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mixer.h"
#include "param.h"
#include "mode.h"

int32_t _GPIO_outputs[8];
static int32_t prescaled_outputs[8];
int32_t _outputs[8];
command_t _command;
output_type_t _GPIO_output_type[8];

static mixer_t quadcopter_plus_mixing =
{
  {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

  { 1000,  1000,  1000,  1000, 0, 0, 0, 0}, // F Mix
  { 0,    -1000,  1000,  0,    0, 0, 0, 0}, // X Mix
  { 1000,  0,     0,    -1000, 0, 0, 0, 0}, // Y Mix
  { 1000, -1000, -1000,  1000, 0, 0, 0, 0}  // Z Mix
};


static mixer_t quadcopter_x_mixing =
{
  {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

  { 1000, 1000, 1000, 1000,  0, 0, 0, 0}, // F Mix
  {-1000,-1000, 1000, 1000,  0, 0, 0, 0}, // X Mix
  {-1000, 1000,-1000, 1000,  0, 0, 0, 0}, // Y Mix
  { 1000,-1000,-1000, 1000,  0, 0, 0, 0}  // Z Mix
};

static mixer_t quadcopter_h_mixing =
{
  {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

  { 1000, 1000, 1000, 1000,  0, 0, 0, 0}, // F Mix
  {-1057, -943, 1057,  943,  0, 0, 0, 0}, // X Mix
  {-1005,  995,-1005,  995,  0, 0, 0, 0}, // Y Mix
  { 1000,-1000,-1000, 1000,  0, 0, 0, 0}  // Z Mix
};

static mixer_t fixedwing_mixing =
{
  {S, S, M, S, NONE, NONE, NONE, NONE},

  {0,    0,    1000, 0,    0, 0, 0, 0}, // F Mix
  {1000, 0,    0,    0,    0, 0, 0, 0}, // X Mix
  {0,    1000, 0,    0,    0, 0, 0, 0}, // Y Mix
  {0,    0,    0,    1000, 0, 0, 0, 0}  // Z Mix
};

static mixer_t tricopter_mixing =
{
  {M, M, M, S, NONE, NONE, NONE, NONE},

  {1000,  1000, 1000, 0,    0, 0, 0, 0}, // F Mix
  {-1000, 1000, 0,    0,    0, 0, 0, 0}, // X Mix
  {-667,  -667, 1333, 0,    0, 0, 0, 0}, // Y Mix
  {0,     0,    0,    1000, 0, 0, 0, 0}  // Z Mix
};

static mixer_t Y6_mixing =
{
  {M, M, M, M, M, M, NONE, NONE},
  { 1000, 1000, 1000, 1000, 1000, 1000, 0, 0}, // F Mix
  { 0,   -1000, 1000, 0,   -1000, 1000, 0, 0}, // X Mix
  {-1300,  667,  667,-1300,  667,  667, 0, 0}, // Y Mix
  {-1000, 1000, 1000, 1000,-1000,-1000, 0, 0}  // Z Mix
};

static mixer_t mixer_to_use;

static mixer_t *array_of_mixers[5] =
{
  &quadcopter_plus_mixing,
  &quadcopter_x_mixing,
  &quadcopter_h_mixing,
  &tricopter_mixing,
  &Y6_mixing,
  &fixedwing_mixing
};



void init_mixing()
{
  // We need a better way to choosing the mixer
  mixer_to_use = *array_of_mixers[_params.values[PARAM_MIXER]];

  for (int8_t i=0; i<8; i++)
  {
    _outputs[i] = 0;
    prescaled_outputs[i] = 0;
    _GPIO_outputs[i] = 0;
    _GPIO_output_type[i] = NONE;
  }
  _command.F = 0;
  _command.x = 0;
  _command.y = 0;
  _command.z = 0;
}

void init_PWM()
{
  bool useCPPM = _params.values[PARAM_RC_TYPE];
  int16_t motor_refresh_rate = _params.values[PARAM_MOTOR_PWM_SEND_RATE];
  int16_t idle_pwm = _params.values[PARAM_MOTOR_IDLE_PWM];
  pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);
}


void write_motor(uint8_t index, int32_t value)
{
  if (_armed_state == ARMED)
  {
    if (value > 1000)
    {
      value = 1000;
    }
    else if (value < 150 && _params.values[PARAM_SPIN_MOTORS_WHEN_ARMED])
    {
      value = 150;
    }
    else if (value < 0)
    {
      value = 0;
    }
  }
  else
  {
    value = 0;
  }
  _outputs[index] = value + 1000;
  pwmWriteMotor(index, _outputs[index]);
}


void write_servo(uint8_t index, int32_t value)
{
  if (value > 500)
  {
    value = 500;
  }
  else if (value < -500)
  {
    value = -500;
  }
  _outputs[index] = value+1500;
  pwmWriteMotor(index, _outputs[index]);
}


void mix_output()
{
  int32_t max_output = 0;
  for (int8_t i=0; i<8; i++)
  {
    if (mixer_to_use.output_type[i] != NONE)
    {
      // Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
      prescaled_outputs[i] = (_command.F*mixer_to_use.F[i] + _command.x*mixer_to_use.x[i] +
                              _command.y*mixer_to_use.y[i] + _command.z*mixer_to_use.z[i])/1000;
      if (prescaled_outputs[i] > 1000 && prescaled_outputs[i] > max_output)
      {
        max_output = prescaled_outputs[i];
      }
      // negative motor outputs are set to zero when writing to the motor,
      // but they have to be allowed here because the same logic is used for
      // servo commands, which may be negative
    }
  }

  // saturate outputs to maintain controllability even during aggressive maneuvers
  if (max_output > 1000)
  {
    int32_t scale_factor = 1000*1000/max_output;
    for (int8_t i=0; i<8; i++)
    {
      if (mixer_to_use.output_type[i] == M)
      {
        prescaled_outputs[i] = (prescaled_outputs[i])*scale_factor/1000; // divide by scale factor
      }
    }
  }

  // Reverse Fixedwing channels
  if (_params.values[PARAM_FIXED_WING])
  {
    prescaled_outputs[0] *= _params.values[PARAM_AILERON_REVERSE] ? -1 : 1;
    prescaled_outputs[1] *= _params.values[PARAM_ELEVATOR_REVERSE] ? -1 : 1;
    prescaled_outputs[3] *= _params.values[PARAM_RUDDER_REVERSE] ? -1 : 1;
  }

  // Add in GPIO inputs from Onboard Computer
  for (int8_t i=0; i<8; i++)
  {
    output_type_t output_type = mixer_to_use.output_type[i];
    if (output_type == NONE)
    {
      // Incorporate GPIO on not already reserved outputs
      prescaled_outputs[i] = _GPIO_outputs[i];
      output_type = _GPIO_output_type[i];
    }

    // Write output to motors
    if (output_type == S)
    {
      write_servo(i, prescaled_outputs[i]);
    }
    else if (output_type == M)
    {
      write_motor(i, prescaled_outputs[i]);
    }
    // If we need to configure another type of output, do it here
  }
}
#ifdef __cplusplus
}
#endif
