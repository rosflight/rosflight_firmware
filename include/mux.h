#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "arming_fsm.h"
#include "param.h"
#include "board.h"

namespace rosflight
{

class Mux
{
public:
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

  mux_t muxes[4] =
  {
    {&_rc_control.x, &_offboard_control.x, &_combined_control.x},
    {&_rc_control.y, &_offboard_control.y, &_combined_control.y},
    {&_rc_control.z, &_offboard_control.z, &_combined_control.z},
    {&_rc_control.F, &_offboard_control.F, &_combined_control.F}
  };


  control_t _rc_control;
  control_t _offboard_control;
  control_t _combined_control;
  control_t _failsafe_control =
  {
    {true, ANGLE, 0.0},
    {true, ANGLE, 0.0},
    {true, RATE, 0.0},
    {true, THROTTLE, 0.0}
  };

  bool _new_command;

  bool mux_inputs();
  void init(Arming_FSM* _fsm, Params* _params, Board* _board);

private:
  Board* board;
  Arming_FSM* fsm;
  Params* params;

  void do_muxing(uint8_t mux_channel);
  void do_min_throttle_muxing();
};

}
