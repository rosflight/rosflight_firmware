#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "board.h"
#include "mux.h"
#include "param.h"
#include "arming_fsm.h"

namespace rosflight {

class Mixer
{

public:
  typedef enum
  {
    QUADCOPTER_PLUS,
    QUADCOPTER_X,
    QUADCOPTER_H,
    TRICOPTER,
    Y6,
    FIXEDWING,
    MYTWINDREAM_MIXER,
    NUM_MIXERS

  } mixer_type_t;

  typedef enum
  {
    NONE, // None
    S, // Servo
    M, // Motor
    G // GPIO
  } output_type_t;

  typedef struct
  {
    float F;
    float x;
    float y;
    float z;
  } command_t;

  typedef struct
  {
    output_type_t output_type[8];
    float F[8];
    float x[8];
    float y[8];
    float z[8];
  } mixer_t;

  command_t _command;

  float _GPIO_outputs[8];
  output_type_t _GPIO_output_type[8];

  float _outputs[8];

  void init(Board* _board, Mux* _mux, Params* _params, Arming_FSM* _fsm);
  void init_PWM();
  void init_mixing();
  void mix_output();

private:

  Board* board;
  Mux* mux;
  Params* params;
  Arming_FSM* fsm;

  float prescaled_outputs[8];

  void write_motor(uint8_t index, float value);
  void write_servo(uint8_t index, float value);

  const mixer_t quadcopter_plus_mixing =
  {
    {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

    { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    { 0.0f, -1.0f,  1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    {-1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    {-1.0f,  1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };


  const mixer_t quadcopter_x_mixing =
  {
    {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

    { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    {-1.0f,-1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    {-1.0f, 1.0f,-1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    {-1.0f, 1.0f, 1.0f,-1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t quadcopter_h_mixing =
  {
    {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

    { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    {-1057, -943, 1057,  943,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    {-1005,  995,-1005,  995,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    {-1.0f, 1.0f, 1.0f,-1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t fixedwing_mixing =
  {
    {S, S, M, S, S, M, NONE, NONE},

    { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    { 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t mytwindream_mixing =
  {
    {S, S, M, S, S, M, NONE, NONE},

    { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    { 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t tricopter_mixing =
  {
    {M, M, M, S, NONE, NONE, NONE, NONE},

    { 1.0f,     1.0f,   1.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
    {-1.0f,     1.0f,   0.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
    {-0.667f,  -0.667f, 1.333f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
    { 0.0f,     0.0f,   0.0f,   1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
  };

  const mixer_t Y6_mixing =
  {
    {M, M, M, M, M, M, NONE, NONE},
    { 1.0f,   1.0f,    1.0f,    1.0f,    1.0f,    1.0f,   0.0f, 0.0f}, // F Mix
    { 0.0f,  -1.0f,    1.0f,    0.0f,   -1.0f,    1.0f,   0.0f, 0.0f}, // X Mix
    {-1.333f, 0.667f,  0.667f, -1.333f,  0.667f,  0.667f, 0.0f, 0.0f}, // Y Mix
    {-1.0f,   1.0f,    1.0f,    1.0f,   -1.0f,   -1.0f,   0.0f, 0.0f}  // Z Mix
  };

  const mixer_t* mixer_to_use;

  const mixer_t *array_of_mixers[NUM_MIXERS] =
  {
    &quadcopter_plus_mixing,
    &quadcopter_x_mixing,
    &quadcopter_h_mixing,
    &tricopter_mixing,
    &Y6_mixing,
    &fixedwing_mixing,
    &mytwindream_mixing
  };


};

}
