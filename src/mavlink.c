#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_receive.h"
#include "param.h"

#include "mavlink.h"

// global variable definitions
mavlink_system_t mavlink_system;

// function definitions
void init_mavlink(void)
{
  // Initialize Serial ports
  mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
  mavlink_system.compid = 250;

  _offboard_control_time = 0;
}

// implement for mavlink convenience functions
inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
  if (chan == MAVLINK_COMM_0)
  {
    serialWrite(Serial1, ch);
  }
}
