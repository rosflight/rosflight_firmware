#include <stdint.h>

#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_serial.h>

#include "mavlink_bridge.h"

inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
  if (chan == MAVLINK_COMM_0)
  {
    serialWrite(Serial1, ch);
  }
}
