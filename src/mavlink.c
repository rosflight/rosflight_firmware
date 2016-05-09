#include <mavlink/v1.0/common/mavlink.h>

#include "param.h"

#include "mavlink.h"

// local definitions
#define MAVLINK_HIGHRES_IMU_FIELDS 0xFC00

// local function definitions
static void send_message(const mavlink_message_t msg, uint16_t len)
{
  (void)msg;
  (void)len;
}

// function definitions
void send_imu(uint64_t time_usec, float ax, float ay, float az, float gx, float gy, float gz)
{
  mavlink_message_t msg;
  uint16_t len = mavlink_msg_highres_imu_pack(_params.mavlink.system_id, _params.mavlink.component_id, &msg,
    time_usec, ax, ay, az, gx, gy, gz, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, MAVLINK_HIGHRES_IMU_FIELDS);
  send_message(msg, len);
}
