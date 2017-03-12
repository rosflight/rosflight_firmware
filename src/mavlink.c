#include <stdint.h>

#include "board.h"
#include "mavlink_receive.h"
#include "param.h"

#include "mavlink.h"
#include "mavlink_log.h"

// global variable definitions
mavlink_system_t mavlink_system;

// function definitions
void init_mavlink(void)
{
  serial_init(get_param_int(PARAM_BAUD_RATE));

  mavlink_system.sysid = get_param_int(PARAM_SYSTEM_ID);
  mavlink_system.compid = 250;

  _offboard_control_time = 0;

  mavlink_log_warning("rebooting", NULL);
}

// implement for mavlink convenience functions
inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
  if (chan == MAVLINK_COMM_0)
  {
    serial_write(ch);
  }
}
