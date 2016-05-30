#include <breezystm32/breezystm32.h>

typedef enum
{
  RC_RATE, // Channel is is in rate mode (mrad/s)
  RC_ANGLE, // Channel command is in angle mode (mrad)
  RC_THROTTLE, // Channel is direcly controlling throttle max/1000
  RC_ALTITUDE, // Channel is commanding a specified altitude
  RC_MIN_THROTTLE, // Channel will be compared with the offboard control input and the min throttle command will be taken
  RC_MIN_THROTTLE_ALTITUDE, // Channel will be the min throttle between command to maintain commanded altitude, and offboard throttle command
  RC_PASSTHROUGH, // Channel directly passes PWM input to the mixer
  RC_OFFBOARD // Channel is not being controlled by RC, but instead by offboard computer
} rc_control_type_t;

typedef struct
{
  rc_control_type_t x_type;
  rc_control_type_t y_type;
  rc_control_type_t z_type;
  rc_control_type_t F_type;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t F;
} rc_control_t;

extern rc_control_t _rc_commands;

void init_rc(void);
void receive_rc(void);
