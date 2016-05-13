#include "mavlink_bridge.h"
#include "param.h"

#include "mavlink.h"

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

void send_imu(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  mavlink_msg_small_imu_send(MAVLINK_COMM_0, ax, ay, az, gx, gy, gz);
}
