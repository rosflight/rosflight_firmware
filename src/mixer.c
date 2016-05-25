#include <stdint.h>

#include <breezystm32.h>

#include "mixer.h"

int32_t _GPIO_outputs[8];
int32_t _outputs[8];
command_t _command;
output_type_t _GPIO_output_type[8];

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
  mixer_to_use = *array_of_mixers[QUADCOPTER_X];

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


void write_motor(uint8_t index, int32_t value){
  /** TODO:
   * reverse the PWM -> mRad conversion in RC
   */

  if (value > 1000)
  {
    value = 1000;
  }
  else if (value < 0)
  {
    value = 0;
  }
  pwmWriteMotor(index, value+2000);
}

void write_servo(uint8_t index, int32_t value){
  /** TODO:
   * reverse the PWM conversion in RC (if any)
   */

  if (value > 500)
  {
    value = 500;
  }
  else if (value < -500)
  {
    value = -500;
  }
  pwmWriteMotor(index, 1500+value);
}

void mix_output()
{
  printf("COMMANDS: F = %d, x = %d, y = %d, z = %d\n", _command.F, _command.x, _command.y, _command.z);
  // Mix Output
  int32_t max_output = 0;
  for (int8_t i=0; i<8; i++)
  {
    if (mixer_to_use.output_type[i] != NONE)
    {
      // Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
      _outputs[i] = (_command.F*mixer_to_use.F[i] + _command.x*mixer_to_use.x[i] +
                    _command.y*mixer_to_use.y[i] + _command.z*mixer_to_use.z[i])/1000;
      if (_outputs[i] > 1000 && _outputs[i] > max_output)
      {
        max_output = _outputs[i];
      }
      printf("mixer col %d: %d %d %d %d\t", i, mixer_to_use.F[i], mixer_to_use.x[i], mixer_to_use.y[i], mixer_to_use.z[i]);
    }
  }

  // saturate outputs to maintain controllability even at high levels of throttle
  if (max_output > 1000)
  {
    int32_t scale_factor = (max_output*1000)/1000;
    for (int8_t i=0; i<8; i++)
    {
      if ( mixer_to_use.output_type[i] == M)
      {
        _outputs[i] = (_outputs[i]*1000)/scale_factor; // divide by scale factor
      }
    }
  }

  // Add in GPIO inptus from Onboard Computer
  for ( int8_t i=0; i<8; i++)
  {
    output_type_t output_type = mixer_to_use.output_type[i];
    if ( output_type == NONE)
    {
      // Incorporate GPIO on not already reserved outputs
      _outputs[i] = _GPIO_outputs[i];
      output_type = _GPIO_output_type[i];
    }

    // Write output to motors
    if (output_type == S)
    {
      write_servo(i, _outputs[i]);
      printf("writing SERVO %d, %d\n", i+1, _outputs[i]);
    }
    else if (output_type == M)
    {
      write_motor(i, _outputs[i]);
      printf("writing MOTOR %d: %d\n", i+1, _outputs[i]);
    }
    // If we need to configure another type of output, do it here
  }
}
