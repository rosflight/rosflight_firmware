#include <stdint.h>

#include <breezystm32/drv_pwm.h>

#include "mixer.h"

int16_t _GPIO_outputs[8];
int16_t _outputs[8];
command_t _command;

static mixer_t quadcopter_plus_mixing = {
  {M, M, M, M, 0, 0, 0, 0}, // output_type

  { 1000, 1000,  1000, 1000, 0, 0, 0, 0}, // F Mix
  { 0,   -1000,  1000, 0,    0, 0, 0, 0}, // X Mix
  {-1000, 0,     0,    1000, 0, 0, 0, 0}, // Y Mix
  {-1000, 1000,  1000,-1000, 0, 0, 0, 0}  // Z Mix
};


static mixer_t quadcopter_x_mixing = {
  {M, M, M, M, 0, 0, 0, 0}, // output_type

  { 1000, 1000, 1000, 1000,  0, 0, 0, 0}, // F Mix
  {-1000,-1000, 1000, 1000,  0, 0, 0, 0}, // X Mix
  {-1000, 1000,-1000, 1000,  0, 0, 0, 0}, // Y Mix
  { 1000,-1000,-1000, 1000,  0, 0, 0, 0}  // Z Mix
};

static mixer_t fixedwing_mixing = {
  {M, S, S, S, 0, 0, 0, 0},

  {1000, 0,    0,    0,    0, 0, 0, 0}, // F Mix
  {0,    1000, 0,    0,    0, 0, 0, 0}, // X Mix
  {0,    0,    1000, 0,    0, 0, 0, 0}, // Y Mix
  {0,    0,    0,    1000, 0, 0, 0, 0}  // Z Mix
};

static mixer_t tricopter_mixing = {
  {M, M, M, S, 0, 0, 0, 0},

  {1000,  1000, 1000, 0,    0, 0, 0, 0}, // F Mix
  {-1000, 1000, 0,    0,    0, 0, 0, 0}, // X Mix
  {-667,  -667, 1333, 0,    0, 0, 0, 0}, // Y Mix
  {0,     0,    0,    1000, 0, 0, 0, 0}  // Z Mix
};

static mixer_t Y6_mixing = {
  {M, M, M, M, M, M, 0, 0},
  { 1000, 1000, 1000, 1000, 1000, 1000, 0, 0}, // F Mix
  { 0,   -1000, 1000, 0,   -1000, 1000, 0, 0}, // X Mix
  {-1300,  667,  667,-1300,  667,  667, 0, 0}, // Y Mix
  {-1000, 1000, 1000, 1000,-1000,-1000, 0, 0}  // Z Mix
};

static mixer_t mixer_to_use;

static mixer_t *array_of_mixers[5] = {
  &quadcopter_plus_mixing,
  &quadcopter_x_mixing,
  &tricopter_mixing,
  &Y6_mixing,
  &fixedwing_mixing
};



void init_mixing()
{
  // We need a better way to choosing the mixer
  mixer_to_use = *array_of_mixers[FIXEDWING];

  for(int8_t i=0; i<8; i++)
  {
    _outputs[i] = 0;
    _GPIO_outputs[i] = 0;
    _GPIO_output_type[i] = 0;
  }
  _command.F = 0;
  _command.x = 0;
  _command.y = 0;
  _command.z = 0;
}

void mix_output()
{
  // Mix Output
  for (int8_t i=0; i<8; i++)
  {
    output_type_t output_type = mixer_to_use.output_type[i];
    if(output_type != NONE)
    {
      // Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
      _outputs[i] = (_command.F*mixer_to_use.F[i] + _command.x*mixer_to_use.x[i] +
                    _command.y*mixer_to_use.y[i] + _command.z*mixer_to_use.z[i])/1000;
    }
    else
    {
      // Incorporate GPIO on not already reserved outputs
      _outputs[i] = _GPIO_outputs[i];
      output_type = _GPIO_output_type[i];
    }

    if (output_type == S)
    {
      write_servo(i, _outputs[i]);
    }
    else
    {
      write_motor(i, _outputs[i]);
    }
    // If we need to configure another type of output, do it here
  }
}


void write_motor(uint8_t index, int16_t value){
  /** TODO:
   * reverse the PWM -> mRad conversion in RC
   */

  if (value > 2000)
  {
    value = 2000;
  }
  else if (value < 0)
  {
    value = 0;
  }
  pwmWriteMotor(index, value);
}

void write_servo(uint8_t index, int16_t value){
  /** TODO:
   * reverse the PWM conversion in RC (if any)
   */

  if (value > 2000)
  {
    value = 2000;
  }
  else if (value < 0)
  {
    value = 0;
  }
  pwmWriteMotor(index, value);
}
