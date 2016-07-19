#ifdef __cplusplus
extern "C" {
#endif


#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  RATE, // Channel is is in rate mode (mrad/s)
  ANGLE, // Channel command is in angle mode (mrad)
  THROTTLE, // Channel is direcly controlling throttle max/1000
  ALTITUDE, // Channel is commanding a specified altitude in cm
  PASSTHROUGH, // Channel directly passes PWM input to the mixer
} control_type_t;

typedef struct
{
  bool active; // Whether or not the channel is active
  control_type_t type;  // What type the channel is
  float value; // The value of the channel
} control_channel_t;

typedef struct
{
  control_channel_t x;
  control_channel_t y;
  control_channel_t z;
  control_channel_t F;
} control_t;

extern control_t _rc_control;
extern control_t _offboard_control;
extern control_t _combined_control;

extern bool _new_command;

bool mux_inputs();

#ifdef __cplusplus
}
#endif

