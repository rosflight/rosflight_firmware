#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wswitch"

#include <stdint.h>

#include "param.h"

#include "mavlink_param.h"

// local variable definitions
static uint8_t send_params_index = PARAMS_COUNT; // current param to send when sending parameter list with low priority

// function definitions
void mavlink_send_param(param_id_t id)
{
  if (id < PARAMS_COUNT)
  {
    MAV_PARAM_TYPE type;
    switch (_params.types[id])
    {
    case PARAM_TYPE_INT32:
      type = MAV_PARAM_TYPE_INT32;
      break;
    case PARAM_TYPE_FLOAT:
      type = MAV_PARAM_TYPE_REAL32;
      break;
    default:
      return;
    }

    mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                 _params.names[id], get_param_float(id), type, PARAMS_COUNT, id);
  }
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
    param_id_t id = (read.param_index < 0) ? lookup_param_id(read.param_id) : (param_id_t) read.param_index;

    if (id < PARAMS_COUNT)
      mavlink_send_param(id);
  }
}

void mavlink_handle_msg_param_set(const mavlink_message_t *const msg)
{
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  if (set.target_system == (uint8_t) _params.values[PARAM_SYSTEM_ID]) // TODO check if component id matches?
  {
    param_id_t id = lookup_param_id(set.param_id);

    if (id < PARAMS_COUNT)
    {
      param_type_t candidate_type;
      switch (set.param_type)
      {
      case MAV_PARAM_TYPE_INT32:
        candidate_type = PARAM_TYPE_INT32;
        break;
      case MAV_PARAM_TYPE_REAL32:
        candidate_type = PARAM_TYPE_FLOAT;
        break;
      default:
        candidate_type = PARAM_TYPE_INVALID;
        break;
      }

      if (candidate_type == _params.types[id])
      {
        switch (candidate_type)
        {
        case PARAM_TYPE_INT32:
          set_param_by_id(id, *(int32_t *) &set.param_value);
          break;
        case PARAM_TYPE_FLOAT:
          set_param_by_id_float(id, set.param_value);
          break;
        }
      }
    }
  }
}

void mavlink_send_next_param(void)
{
  if (send_params_index < PARAMS_COUNT)
  {
    mavlink_send_param((param_id_t) send_params_index);
    send_params_index++;
  }
}
