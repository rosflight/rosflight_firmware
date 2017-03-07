#include "board.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mode.h"
#include "param.h"
#include "mux.h"
#include "sensors.h"
#include "rc.h"

#include "mavlink_receive.h"
#include "mavlink_log.h"

#include "mavlink_util.h"

// global variable definitions
mavlink_offboard_control_t mavlink_offboard_control;
uint64_t _offboard_control_time;

// local variable definitions
static mavlink_message_t in_buf;
static mavlink_status_t status;

// local function definitions
static void mavlink_handle_msg_rosflight_cmd(const mavlink_message_t *const msg)
{
  mavlink_rosflight_cmd_t cmd;
  mavlink_msg_rosflight_cmd_decode(msg, &cmd);

  uint8_t result;
  bool reboot_flag = false;
  bool reboot_to_bootloader_flag = false;

  // None of these actions can be performed if we are armed
  if (_armed_state == ARMED)
  {
    result = false;
  }
  else
  {
    result = true;
    switch (cmd.command)
    {
    case ROSFLIGHT_CMD_READ_PARAMS:
      result = read_params();
      break;
    case ROSFLIGHT_CMD_WRITE_PARAMS:
      result = write_params();
      break;
    case ROSFLIGHT_CMD_SET_PARAM_DEFAULTS:
      set_param_defaults();
      break;
    case ROSFLIGHT_CMD_ACCEL_CALIBRATION:
      result = start_imu_calibration();
      break;
    case ROSFLIGHT_CMD_GYRO_CALIBRATION:
      result = start_gyro_calibration();
      break;
    case ROSFLIGHT_CMD_BARO_CALIBRATION:
      baro_calibrate();
      break;
    case ROSFLIGHT_CMD_AIRSPEED_CALIBRATION:
      diff_pressure_calibrate();
      break;
    case ROSFLIGHT_CMD_RC_CALIBRATION:
      _calibrate_rc = true;
      break;
    case ROSFLIGHT_CMD_REBOOT:
      reboot_flag = true;
      break;
    case ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER:
      reboot_to_bootloader_flag = true;
      break;
    default:
      mavlink_log_error("unsupported ROSFLIGHT CMD %d", cmd.command);
      result = false;
      break;
    }
  }

  uint8_t response = (result) ? ROSFLIGHT_CMD_SUCCESS : ROSFLIGHT_CMD_FAILED;

  mavlink_msg_rosflight_cmd_ack_send(MAVLINK_COMM_0, cmd.command, response);

  if (reboot_flag || reboot_to_bootloader_flag)
  {
    clock_delay(20);
    board_reset(reboot_to_bootloader_flag);
  }
}

static void mavlink_handle_msg_timesync(const mavlink_message_t *const msg)
{
  uint64_t now_us = clock_micros();

  mavlink_timesync_t tsync;
  mavlink_msg_timesync_decode(msg, &tsync);

  if (tsync.tc1 == 0) // check that this is a request, not a response
  {
    mavlink_msg_timesync_send(MAVLINK_COMM_0, (int64_t) now_us*1000, tsync.ts1);
  }
}

static void mavlink_handle_msg_offboard_control(const mavlink_message_t *const msg)
{
  _offboard_control_time = clock_micros();
  mavlink_msg_offboard_control_decode(msg, &mavlink_offboard_control);

  // put values into standard message (Commands coming in are in NED, onboard estimator is in NWU)
  _offboard_control.x.value = mavlink_offboard_control.x;
  _offboard_control.y.value = -mavlink_offboard_control.y;
  _offboard_control.z.value = -mavlink_offboard_control.z;
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
    _offboard_control.x.value = mavlink_offboard_control.x*500.0f + (float)get_param_int(PARAM_RC_X_CENTER) - 1500.0f;
    _offboard_control.y.value = mavlink_offboard_control.y*500.0f + (float)get_param_int(PARAM_RC_Y_CENTER) - 1500.0f;
    _offboard_control.z.value = mavlink_offboard_control.z*500.0f + (float)get_param_int(PARAM_RC_Z_CENTER) - 1500.0f;
    _offboard_control.F.value = mavlink_offboard_control.F*1000.0f;
    break;
  case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    _offboard_control.x.type = RATE;
    _offboard_control.y.type = RATE;
    _offboard_control.z.type = RATE;
    _offboard_control.F.type = THROTTLE;
    _offboard_control.F.value = mavlink_offboard_control.F*1000.0f;
    _offboard_control.x.value += get_param_float(PARAM_ROLL_RATE_TRIM);
    _offboard_control.y.value += get_param_float(PARAM_PITCH_RATE_TRIM);
    _offboard_control.z.value += get_param_float(PARAM_YAW_RATE_TRIM);
    break;
  case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
    _offboard_control.x.type = ANGLE;
    _offboard_control.y.type = ANGLE;
    _offboard_control.z.type = RATE;
    _offboard_control.F.type = THROTTLE;
    _offboard_control.F.value = mavlink_offboard_control.F*1000.0f;
    _offboard_control.x.value += get_param_float(PARAM_ROLL_ANGLE_TRIM);
    _offboard_control.y.value += get_param_float(PARAM_PITCH_ANGLE_TRIM);
    _offboard_control.z.value += get_param_float(PARAM_YAW_RATE_TRIM);
    break;
  case MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
    _offboard_control.x.type = ANGLE;
    _offboard_control.y.type = ANGLE;
    _offboard_control.z.type = RATE;
    _offboard_control.F.type = ALTITUDE;
    _offboard_control.x.value += get_param_float(PARAM_ROLL_ANGLE_TRIM);
    _offboard_control.y.value += get_param_float(PARAM_PITCH_ANGLE_TRIM);
    _offboard_control.z.value += get_param_float(PARAM_YAW_RATE_TRIM);
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
  case MAVLINK_MSG_ID_ROSFLIGHT_CMD:
    mavlink_handle_msg_rosflight_cmd(&in_buf);
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
  while (serial_bytes_available())
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, serial_read(), &in_buf, &status))
      handle_mavlink_message();
  }
}
