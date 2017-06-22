/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rosflight.h"
#include "mavlink.h"

namespace rosflight_firmware{


// local function definitions
void Mavlink::mavlink_handle_msg_rosflight_cmd(const mavlink_message_t *const msg)
{
  mavlink_rosflight_cmd_t cmd;
  mavlink_msg_rosflight_cmd_decode(msg, &cmd);

  uint8_t result;
  bool reboot_flag = false;
  bool reboot_to_bootloader_flag = false;

  // None of these actions can be performed if we are armed
  if (RF_->state_manager_.state().armed)
  {
    result = false;
  }
  else
  {
    result = true;
    switch (cmd.command)
    {
    case ROSFLIGHT_CMD_READ_PARAMS:
      result = RF_->params_.read_params();
      break;
    case ROSFLIGHT_CMD_WRITE_PARAMS:
      result = RF_->params_.write_params();
      break;
    case ROSFLIGHT_CMD_SET_PARAM_DEFAULTS:
      RF_->params_.set_param_defaults();
      break;
    case ROSFLIGHT_CMD_ACCEL_CALIBRATION:
      result = RF_->sensors_.start_imu_calibration();
      break;
    case ROSFLIGHT_CMD_GYRO_CALIBRATION:
      result = RF_->sensors_.start_gyro_calibration();
      break;
    case ROSFLIGHT_CMD_BARO_CALIBRATION:
      RF_->board_.baro_calibrate();
      break;
    case ROSFLIGHT_CMD_AIRSPEED_CALIBRATION:
      RF_->board_.diff_pressure_calibrate();
      break;
    case ROSFLIGHT_CMD_RC_CALIBRATION:
      RF_->controller_.calculate_equilbrium_torque_from_rc();
      break;
    case ROSFLIGHT_CMD_REBOOT:
      reboot_flag = true;
      break;
    case ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER:
      reboot_to_bootloader_flag = true;
      break;
    case ROSFLIGHT_CMD_SEND_VERSION:
      mavlink_message_t msg;
      mavlink_msg_rosflight_version_pack(sysid, compid, &msg, GIT_VERSION_STRING);
      send_message(msg);
      break;
    default:
      // log_error(this, "unsupported ROSFLIGHT CMD %d", cmd.command);
      result = false;
      break;
    }
  }

  uint8_t response = (result) ? ROSFLIGHT_CMD_SUCCESS : ROSFLIGHT_CMD_FAILED;

  mavlink_message_t ack_msg;
  mavlink_msg_rosflight_cmd_ack_pack(sysid, compid, &ack_msg, cmd.command, response);
  send_message(ack_msg);

  if (reboot_flag || reboot_to_bootloader_flag)
  {
    RF_->board_.clock_delay(20);
    RF_->board_.board_reset(reboot_to_bootloader_flag);
  }
}

void Mavlink::mavlink_handle_msg_timesync(const mavlink_message_t *const msg)
{
  uint64_t now_us = RF_->board_.clock_micros();

  mavlink_timesync_t tsync;
  mavlink_msg_timesync_decode(msg, &tsync);

  if (tsync.tc1 == 0) // check that this is a request, not a response
  {
    mavlink_message_t msg;
    mavlink_msg_timesync_pack(sysid, compid, &msg, (int64_t) now_us*1000, tsync.ts1);
    send_message(msg);
  }
}

void Mavlink::mavlink_handle_msg_offboard_control(const mavlink_message_t *const msg)
{
  mavlink_offboard_control_t mavlink_offboard_control;
  _offboard_control_time = RF_->board_.clock_micros();
  mavlink_msg_offboard_control_decode(msg, &mavlink_offboard_control);

  // put values into standard message
  RF_->mux_._offboard_control.x.value = mavlink_offboard_control.x;
  RF_->mux_._offboard_control.y.value = mavlink_offboard_control.y;
  RF_->mux_._offboard_control.z.value = mavlink_offboard_control.z;
  RF_->mux_._offboard_control.F.value = mavlink_offboard_control.F;

  // Move flags into standard message
  RF_->mux_._offboard_control.x.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE1);
  RF_->mux_._offboard_control.y.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE2);
  RF_->mux_._offboard_control.z.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE3);
  RF_->mux_._offboard_control.F.active = !(mavlink_offboard_control.ignore & IGNORE_VALUE4);

  // translate modes into standard message
  switch (mavlink_offboard_control.mode)
  {
  case MODE_PASS_THROUGH:
    RF_->mux_._offboard_control.x.type = PASSTHROUGH;
    RF_->mux_._offboard_control.y.type = PASSTHROUGH;
    RF_->mux_._offboard_control.z.type = PASSTHROUGH;
    RF_->mux_._offboard_control.F.type = THROTTLE;
    break;
  case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    RF_->mux_._offboard_control.x.type = RATE;
    RF_->mux_._offboard_control.y.type = RATE;
    RF_->mux_._offboard_control.z.type = RATE;
    RF_->mux_._offboard_control.F.type = THROTTLE;
    break;
  case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
    RF_->mux_._offboard_control.x.type = ANGLE;
    RF_->mux_._offboard_control.y.type = ANGLE;
    RF_->mux_._offboard_control.z.type = RATE;
    RF_->mux_._offboard_control.F.type = THROTTLE;
    break;
    // Handle error state
  }

  // Tell the mux that we have a new command we need to mux
  RF_->mux_.signal_new_command();
}

void Mavlink::handle_mavlink_message(void)
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
void Mavlink::receive(void)
{
  while (RF_->board_.serial_bytes_available())
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, RF_->board_.serial_read(), &in_buf, &status))
      handle_mavlink_message();
  }
}
}
