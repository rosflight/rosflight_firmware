#ifdef __cplusplus
extern "C" {
#endif

#pragma once

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
  QUADCOPTER_PLUS,
  QUADCOPTER_X,
  QUADCOPTER_H,
  TRICOPTER,
  Y6,
  X8,
  FIXEDWING,
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

extern command_t _command;

extern float _GPIO_outputs[8];
extern output_type_t _GPIO_output_type[8];

extern float _outputs[10];

void init_PWM();
void init_mixing();
void mix_output();
#ifdef __cplusplus
}
#endif
