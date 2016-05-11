#include "mavlink_bridge.h"
#include "param.h"

#include "mavlink.h"

// local definitions
#define MAVLINK_HIGHRES_IMU_FIELDS 0xFC00

// function definitions
void init_mavlink(void)
{
  mavlink_system.sysid = _params.mavlink.system_id;
  mavlink_system.compid = _params.mavlink.component_id;
}

void send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_STANDBY);
}

void send_imu(uint64_t time_usec, float ax, float ay, float az, float gx, float gy, float gz)
{
  mavlink_message_t msg;
  uint16_t len = mavlink_msg_highres_imu_pack(_params.mavlink.system_id, _params.mavlink.component_id, &msg,
    time_usec, ax, ay, az, gx, gy, gz, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, MAVLINK_HIGHRES_IMU_FIELDS);
  send_message(msg, len);
}
