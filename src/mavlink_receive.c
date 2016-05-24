#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mode.h"
#include "param.h"

#include "mavlink_receive.h"

// local variable definitions
static mavlink_message_t in_buf;
static mavlink_status_t status;

// local function definitions
static void mavlink_handle_msg_command_int(const mavlink_message_t *const msg)
{
  mavlink_command_int_t cmd;
  mavlink_msg_command_int_decode(msg, &cmd);

  if (cmd.target_system == _params.values[PARAM_SYSTEM_ID])
  {
    uint8_t result;

    switch (cmd.command)
    {
    case MAV_CMD_PREFLIGHT_STORAGE:
      if (false) //TODO temporarily reject if armed
      {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
      }
      else
      {
        bool success;
        switch ((uint8_t) cmd.param1)
        {
        case 0:
          success = read_params();
          break;
        case 1:
          success = write_params();
          break;
        case 2:
          set_param_defaults();
          success = true;
          break;
        default:
          success = false;
          break;
        }
        result = success ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
      }
      break;
    default:
      result = MAV_RESULT_UNSUPPORTED;
      break;
    }

    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, result);
  }
}

static void handle_mavlink_message(void)
{
  switch (in_buf.msgid)
  {
  case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
    break;
  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    mavlink_handle_msg_param_request_list();
    break;
  case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    mavlink_handle_msg_param_request_read(&in_buf);
    break;
  case MAVLINK_MSG_ID_PARAM_SET:
    mavlink_handle_msg_param_set(&in_buf);
    break;
  case MAVLINK_MSG_ID_COMMAND_INT:
    mavlink_handle_msg_command_int(&in_buf);
    break;
  default:
    break;
  }
}

// function definitions
void mavlink_receive(void)
{
  while (serialTotalBytesWaiting(Serial1))
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, serialRead(Serial1), &in_buf, &status))
      handle_mavlink_message();
  }
}
