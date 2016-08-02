#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mode.h"
#include "param.h"
#include "mux.h"
#include "sensors.h"

#include "mavlink_receive.h"

// global variable definitions
mavlink_offboard_control_t mavlink_offboard_control;
uint32_t _offboard_control_time;

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
      if (_armed_state == ARMED)
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

      // Perform an IMU calibration (static offset calculation)
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      if (_armed_state == ARMED)
      {
        result = MAV_RESULT_TEMPORARILY_REJECTED;
      }
      else
      {
        bool success = false;
        if (cmd.param1)
        {
          success &= calibrate_gyro();
        }
        if (cmd.x) // x is PARAM5
        {
          success &= calibrate_acc();
        }
        result = MAV_RESULT_ACCEPTED;
      }
      break;

    default:
      result = MAV_RESULT_UNSUPPORTED;
      break;
    }

    mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, result);
  }
}

static void mavlink_handle_msg_timesync(const mavlink_message_t *const msg)
{
  uint32_t now_us = micros();

  mavlink_timesync_t tsync;
  mavlink_msg_timesync_decode(msg, &tsync);

  if (tsync.tc1 == 0) // check that this is a request, not a response
  {
    mavlink_msg_timesync_send(MAVLINK_COMM_0, (int64_t) now_us*1000, tsync.ts1);
  }
}

static void mavlink_handle_msg_offboard_control(const mavlink_message_t *const msg)
{
  _offboard_control_time = micros();
  mavlink_msg_offboard_control_decode(msg, &mavlink_offboard_control);

  // put values into standard message
  _offboard_control.x.value = mavlink_offboard_control.x;
  _offboard_control.y.value = mavlink_offboard_control.y;
  _offboard_control.z.value = mavlink_offboard_control.z;
  _offboard_control.F.value = mavlink_offboard_control.F;

  // Move flags into standard message
  _offboard_control.x.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE1);
  _offboard_control.y.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE2);
  _offboard_control.z.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE3);
  _offboard_control.F.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE4);

  // translate modes into standard message
  switch (mavlink_offboard_control.mode)
  {
  case MODE_PASS_THROUGH:
    _offboard_control.x.type = PASSTHROUGH;
    _offboard_control.y.type = PASSTHROUGH;
    _offboard_control.z.type = PASSTHROUGH;
    _offboard_control.F.type = THROTTLE;
    _offboard_control.x.value = mavlink_offboard_control.x/2.0f;
    _offboard_control.y.value = mavlink_offboard_control.y/2.0f;
    _offboard_control.z.value = mavlink_offboard_control.z/2.0f;
    _offboard_control.F.value = mavlink_offboard_control.F;
    break;
  case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    _offboard_control.x.type = RATE;
    _offboard_control.y.type = RATE;
    _offboard_control.z.type = RATE;
    _offboard_control.F.type = THROTTLE;
    break;
  case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
    _offboard_control.x.type = ANGLE;
    _offboard_control.y.type = ANGLE;
    _offboard_control.z.type = RATE;
    _offboard_control.F.type = THROTTLE;
    break;
  case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
    _offboard_control.x.type = ANGLE;
    _offboard_control.y.type = ANGLE;
    _offboard_control.z.type = RATE;
    _offboard_control.F.type = ALTITUDE;
    break;
    // Handle error state
  }
  _new_command = true;
}

static void handle_mavlink_message(void)
{
  switch (in_buf.msgid)
  {
  case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
    mavlink_handle_msg_offboard_control(&in_buf);
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
  case MAVLINK_MSG_ID_TIMESYNC:
    mavlink_handle_msg_timesync(&in_buf);
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

