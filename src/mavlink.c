#include <stdint.h>

#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_serial.h>

#include "param.h"

#include "mavlink.h"

// global variable definitions
mavlink_system_t mavlink_system;

// function definitions
void init_mavlink(void)
{
  mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
  mavlink_system.compid = 50;
}

// implement for mavlink convenience functions
inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
  if (chan == MAVLINK_COMM_0)
  {
    serialWrite(Serial1, ch);
  }
}

// TODO delete these functions
void send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_STANDBY);
}

void send_imu(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  mavlink_msg_small_imu_send(MAVLINK_COMM_0, ax, ay, az, gx, gy, gz);
}
