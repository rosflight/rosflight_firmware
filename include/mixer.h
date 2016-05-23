#include <stdint.h>
#include <stdbool.h>

extern int16_t _motor_outputs[8];

typedef enum{
  NONE, // None
  S, // Servo
  M, // Motor
  L // LED
} output_type_t;

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
  int32_t throttle;
} command_t;

typedef struct {
  uint8_t motor_count;
  uint8_t servo_count;
  output_type_t output_type[8];
  int32_t F[8];
  int32_t x[8];
  int32_t y[8];
  int32_t z[8];
} mixer_t;

extern command_t _theta_command;
extern command_t _omega_command;

void init_mixing();
void mix_output();
