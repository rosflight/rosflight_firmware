#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_receive.h"
#include "param.h"

#include "mavlink.h"
#include "mavlink_log.h"

// global variable definitions
mavlink_system_t mavlink_system;
serialPort_t *Serial1;

// function definitions
void init_mavlink(void)
{
  // Initialize Serial ports
  Serial1 = uartOpen(USART1, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX);

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
    serialWrite(Serial1, ch);
  }
}
