#include <stdint.h>

#include "param.h"

#include "mavlink_param.h"

// local variable definitions
static uint8_t send_params_index = PARAMS_COUNT; // current param to send when sending parameter list with low priority

// function definitions
void mavlink_send_param(param_id_t id)
{
  mavlink_msg_param_value_send(MAVLINK_COMM_0,
                               _params.names[id], *(float *) &_params.values[id], MAV_PARAM_TYPE_INT32, PARAMS_COUNT, id);
}

void mavlink_handle_msg_param_request_list(void)
{
  send_params_index = 0;
}

void mavlink_handle_msg_param_request_read(const mavlink_message_t *const msg)
{
  mavlink_param_request_read_t read;
  mavlink_msg_param_request_read_decode(msg, &read);

  if (read.target_system == (uint8_t) _params.values[PARAM_SYSTEM_ID]) // TODO check if component id matches?
  {
    param_id_t id = (read.param_index < 0) ? lookup_param_id(read.param_id) : read.param_index;

    if (id < PARAMS_COUNT)
    {
      mavlink_send_param(id);
    }
  }
}

void mavlink_handle_msg_param_set(const mavlink_message_t *const msg)
{
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  if (set.target_system == (uint8_t) _params.values[PARAM_SYSTEM_ID]) // TODO check if component id matches?
  {
    if (set.param_type == MAV_PARAM_TYPE_INT32) // TODO support other param types? (uint32 at least?)
    {
      param_id_t id = lookup_param_id(set.param_id);
      set_param_by_id(id, *(int32_t*) &set.param_value);
    }
  }
}

void mavlink_send_next_param(void)
{
  if (send_params_index < PARAMS_COUNT)
  {
    mavlink_send_param(send_params_index);
    send_params_index++;
  }
}
