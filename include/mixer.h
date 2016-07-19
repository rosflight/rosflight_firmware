#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
  QUADCOPTER_PLUS,
  QUADCOPTER_X,
  QUADCOPTER_H,
  TRICOPTER,
  Y6,
  FIXEDWING
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
  int32_t F;
  int32_t x;
  int32_t y;
  int32_t z;
} command_t;

typedef struct
{
  output_type_t output_type[8];
  int32_t F[8];
  int32_t x[8];
  int32_t y[8];
  int32_t z[8];
} mixer_t;

extern command_t _command;

extern int32_t _GPIO_outputs[8];
extern output_type_t _GPIO_output_type[8];

extern int32_t _outputs[8];

void init_PWM();
void init_mixing();
void mix_output();
#ifdef __cplusplus
}
#endif
