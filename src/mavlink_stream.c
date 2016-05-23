#include "mavlink.h"
#include "mavlink_param.h"
#include "sensors.h"

#include "mavlink_stream.h"

// local variable definitions
static uint32_t low_priority_period_us = 1e5; // 10 Hz TODO make a param?
static uint32_t last_low_priority_us = 0;

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
    mavlink_msg_small_imu_send(MAVLINK_COMM_0,
      _accel_data[0], _accel_data[1], _accel_data[2], _gyro_data[0], _gyro_data[1], _gyro_data[2]);
  }

  if (time_us - last_low_priority_us >= low_priority_period_us)
  {
    last_low_priority_us = time_us;
    mavlink_send_low_priority();
  }
}

void mavlink_stream_set_heartbeat_period_us(uint32_t period)
{
  heartbeat_period_us = period;
}

void mavlink_stream_set_imu_period_us(uint32_t period)
{
  imu_period_us = period;
}
