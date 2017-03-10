#ifndef COMMON_H
#define COMMON_H

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


typedef enum
{
  MUX_X,
  MUX_Y,
  MUX_Z,
  MUX_F,
} mux_channel_t;

typedef struct
{
  control_channel_t* rc;
  control_channel_t* onboard;
  control_channel_t* combined;
} mux_t;

#endif // COMMON_H
