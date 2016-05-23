#include <stdint.h>

#include "mixing.h"


int16_t _motor_outputs[8];
int16_t _servo_outputs[8];
command_t _theta_command;
command_t _omega_command;
command_t _final_command;

typedef enum{
  QUADCOPTER_PLUS,
  QUADCOPTER_X,
  FIXEDWING
} mixer_type_t;

static mixer_t quadcopter_plus_mixing = {
  4, // Motor Count
  0, // Servo Count
  {M, M, M, M, 0, 0, 0, 0}, // output_type

  {1,   1,  1, 1,  0, 0, 0, 0}, // F Mix
  {0,  -1,  1, 0,  0, 0, 0, 0}, // X Mix
  {-1,  0,  0, 1,  0, 0, 0, 0}, // Y Mix
  {-1,  1,  1, -1, 0, 0, 0, 0}  // Z Mix
};


static mixer_t quadcopter_x_mixing = {
  4, // Motor Count
  0, // Servo Count
  {M, M, M, M, 0, 0, 0, 0}, // output_type

  {1,   1,  1,  1,  0, 0, 0, 0}, // F Mix
  {-1, -1,  1,  1,  0, 0, 0, 0}, // X Mix
  {-1,  1,  -1, 1,  0, 0, 0, 0}, // Y Mix
  {-1,  1,  1, -1,  0, 0, 0, 0}  // Z Mix
};

static mixer_t fixedwing_mixing = {
  1, // Motor Count
  3, // Servo Count
  {M, S, S, S, 0, 0, 0, 0},

  {1, 0, 0, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0}
};

static mixer_t mixer_to_use;

static mixer_t* array_of_mixers[2] = {
  &quadcopter_plus_mixing,
  &quadcopter_x_mixing,
  &fixedwing_mixing
};

void init_mixing(){
  mixer_to_use = *array_of_mixers[FIXEDWING];
}
void mix_output(){}
