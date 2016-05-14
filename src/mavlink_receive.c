#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_serial.h>

#include "mavlink.h"
#include "mavlink_param.h"

#include "mavlink_receive.h"

// local variable definitions
static mavlink_message_t in_buf;
static mavlink_status_t status;

// local function definitions
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
    handle_param_set_msg(&in_buf);
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
