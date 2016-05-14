#include <string.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_bridge.h"
#include "param.h"

#include "mavlink.h"

// local variable definitions
static mavlink_message_t out_buf;

static mavlink_message_t in_buf;
static mavlink_status_t status;

static uint8_t send_params_index = PARAMS_COUNT; // current param to send when sending parameter list with low priority

// local function definitions
static void handle_param_set_msg(void)
{
  // TODO need to handle special case of updating SYS_ID; need to update mavlink_system struct (or just require reboot?)
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(&in_buf, &set);
  if (set.target_system == (uint8_t) _params.values[PARAM_SYSTEM_ID]) // TODO check if component id matches?
  {
    if (set.param_type == MAV_PARAM_TYPE_INT32) // TODO support other param types? (uint32 at least?)
    {
      uint8_t id = lookup_param_id(set.param_id);
      if (set_param_by_id(id, *(int32_t *) &set.param_value))
        mavlink_msg_param_value_send_buf(&out_buf, MAVLINK_COMM_0, _params.names[id], *(float *) &_params.values[id], MAV_PARAM_TYPE_INT32, PARAMS_COUNT, id);
    }
  }
}

static void handle_mavlink_message(void)
{
  switch (in_buf.msgid)
  {
  case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
    break;
  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    start_send_param_list();
    break;
  case MAVLINK_MSG_ID_PARAM_SET:
    handle_param_set_msg();
    break;
  default:
    break;
  }
}

// function definitions
void init_mavlink(void)
{
  mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
  mavlink_system.compid = 50;
}

void mavlink_receive(void)
{
  while (serialTotalBytesWaiting(Serial1))
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, serialRead(Serial1), &in_buf, &status))
      handle_mavlink_message();
  }
}

void start_send_param_list(void)
{
  send_params_index = 0;
}

void send_heartbeat(void)
{
  mavlink_msg_heartbeat_send_buf(&out_buf, MAVLINK_COMM_0, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_STANDBY);
}

void send_imu(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
  mavlink_msg_small_imu_send_buf(&out_buf, MAVLINK_COMM_0, ax, ay, az, gx, gy, gz);
}

void mavlink_send_low_priority(void)
{
  if (send_params_index < PARAMS_COUNT)
  {
    mavlink_msg_param_value_send_buf(&out_buf,
      MAVLINK_COMM_0,
      _params.names[send_params_index],
      *(float *) &_params.values[send_params_index],
      MAV_PARAM_TYPE_INT32,
      PARAMS_COUNT,
      send_params_index);

    send_params_index++;
  }
}
