#pragma once

#include <stdint.h>

#include "board.h"
#include "sensors.h"
#include "param.h"

namespace rosflight
{

typedef enum
{
  DISARMED,
  ARMED,
  DISARMED_FAILSAFE,
  ARMED_FAILSAFE
} armed_state_t;

class Arming_FSM
{
private:
  Board* board_;
  Sensors* sensors_;
  Params* params_;

  uint32_t prev_time_ms;
  uint32_t time_sticks_have_been_in_arming_position_ms = 0;

  bool started_gyro_calibration;

  bool arm(void);
  void disarm(void);
  bool check_failsafe(void);


public:
  Arming_FSM();
  armed_state_t _armed_state;

  void init_mode(Board* _board, Sensors* _sensors, Params* _params);
  bool check_mode(uint64_t now);

};

}
