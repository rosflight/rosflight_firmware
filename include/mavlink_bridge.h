#pragma once

#include <stdint.h>

#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_serial.h>

#include <mavlink/v1.0/mavlink_types.h>

#include "param.h"

mavlink_system_t mavlink_system;

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include <mavlink/v1.0/common/mavlink.h>

inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
  if (chan == MAVLINK_COMM_0)
  {
    serialWrite(Serial1, ch);
  }
}
