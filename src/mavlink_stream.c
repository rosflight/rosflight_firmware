#include "mavlink.h"
#include "mavlink_param.h"

#include "mavlink_stream.h"

// local variable definitions
static uint32_t heartbeat_period_us;
static uint32_t last_heartbeat_us = 0;

static uint32_t imu_period_us;
static uint32_t last_imu_us = 0;

// local function definitions
static void mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}

// function definitions
void mavlink_stream(uint32_t time_us)
{
  if (heartbeat_period_us && time_us - last_heartbeat_us >= heartbeat_period_us)
  {
    last_heartbeat_us = time_us;
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_STANDBY);
  }

  if (imu_period_us && time_us - last_imu_us >= imu_period_us)
  {
    last_imu_us = time_us;
    mavlink_msg_small_imu_send(MAVLINK_COMM_0, 1, 2, 3, 4, 5, 6);
  }

  mavlink_send_low_priority();
}

void mavlink_stream_set_heartbeat_period_us(uint32_t period)
{
  heartbeat_period_us = period;
}

void mavlink_stream_set_imu_period_us(uint32_t period)
{
  imu_period_us = period;
}
