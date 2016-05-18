#include <stdint.h>

#include "param.h"

#include "mavlink_param.h"

// local variable definitions
static uint8_t send_params_index = PARAMS_COUNT; // current param to send when sending parameter list with low priority

// function definitions
void handle_param_set_msg(const mavlink_message_t * const msg)
{
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  // TODO need to handle special case of updating SYS_ID; need to update mavlink_system struct (or just require reboot?)
  if (set.target_system == (uint8_t) _params.values[PARAM_SYSTEM_ID]) // TODO check if component id matches?
  {
    if (set.param_type == MAV_PARAM_TYPE_INT32) // TODO support other param types? (uint32 at least?)
    {
      uint8_t id = lookup_param_id(set.param_id);
      if (set_param_by_id(id, *(int32_t *) &set.param_value))
        mavlink_msg_param_value_send(MAVLINK_COMM_0, _params.names[id], *(float *) &_params.values[id], MAV_PARAM_TYPE_INT32, PARAMS_COUNT, id);
    }
  }
}

void start_send_param_list(void)
{
  send_params_index = 0;
}

void mavlink_send_low_priority(void)
{
  if (send_params_index < PARAMS_COUNT)
  {
    mavlink_msg_param_value_send(MAVLINK_COMM_0,
      _params.names[send_params_index],
      *(float *) &_params.values[send_params_index],
      MAV_PARAM_TYPE_INT32,
      PARAMS_COUNT,
      send_params_index);

    send_params_index++;
  }
}
