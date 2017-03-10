#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "arming_fsm.h"
#include "param.h"
#include "board.h"
#include "common.h"

namespace rosflight
{

class Arming_FSM;

class Mux
{
public:

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
