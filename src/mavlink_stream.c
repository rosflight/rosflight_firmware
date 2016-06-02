#include <stdbool.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "sensors.h"

#include "mavlink_stream.h"

// typedefs
typedef struct
{
  uint32_t period_us;
  uint32_t last_time_us;
  void (*send_function)(void);
} mavlink_stream_t;

// local function definitions
static void mavlink_send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0,
    MAV_STATE_STANDBY);
}

static void mavlink_send_imu(void)
{
  mavlink_msg_small_imu_send(MAVLINK_COMM_0,
    _accel_data[0],
    _accel_data[1],
    _accel_data[2],
    _gyro_data[0],
    _gyro_data[1],
    _gyro_data[2]);
}

static void mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}

// local variable definitions
static mavlink_stream_t mavlink_streams[MAVLINK_STREAM_COUNT] = {
  { .period_us = 1e6, .last_time_us = 0, .send_function = mavlink_send_heartbeat },
  { .period_us = 1e4, .last_time_us = 0, .send_function = mavlink_send_imu },
  { .period_us = 1e5, .last_time_us = 0, .send_function = mavlink_send_low_priority }
};

// function definitions
void mavlink_stream(uint32_t time_us)
{
  for (int i = 0; i < MAVLINK_STREAM_COUNT; i++)
  {
    if (mavlink_streams[i].period_us && time_us - mavlink_streams[i].last_time_us >= mavlink_streams[i].period_us)
    {
      mavlink_streams[i].last_time_us = time_us;
      mavlink_streams[i].send_function();
    }
  }
}

void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate)
{

  mavlink_streams[stream_id].period_us = (rate == 0 ? 0 : 1e6 / rate);
}

void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us)
{
  mavlink_streams[stream_id].period_us = period_us;
}
