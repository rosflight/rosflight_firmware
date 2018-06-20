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
#include <cstdint>

#include "board.h"

#include "mavlink.h"

namespace rosflight_firmware
{

Mavlink::Mavlink(Board& board) :
  board_(board)
{}

void Mavlink::init(uint32_t baud_rate)
{
  board_.serial_init(baud_rate);
  initialized_ = true;
}

void Mavlink::receive(void)
{
  while (board_.serial_bytes_available())
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, board_.serial_read(), &in_buf_, &status_))
      handle_mavlink_message();
  }
}

void Mavlink::send_attitude_quaternion(uint8_t system_id,
                              uint64_t timestamp_us,
                              const turbomath::Quaternion &attitude,
                              const turbomath::Vector &angular_velocity)
{
  mavlink_message_t msg;
  mavlink_msg_attitude_quaternion_pack(system_id, compid_, &msg,
                                       timestamp_us / 1000,
                                       attitude.w,
                                       attitude.x,
                                       attitude.y,
                                       attitude.z,
                                       angular_velocity.x,
                                       angular_velocity.y,
                                       angular_velocity.z);
  send_message(msg);
}

void Mavlink::send_baro(uint8_t system_id, float altitude, float pressure, float temperature)
{
  mavlink_message_t msg;
  mavlink_msg_small_baro_pack(system_id, compid_, &msg, altitude, pressure, temperature);
  send_message(msg);
}

void Mavlink::send_command_ack(uint8_t system_id, Command command, bool success)
{
  ROSFLIGHT_CMD rosflight_cmd = ROSFLIGHT_CMD_ENUM_END;
  switch (command)
  {
  case CommLink::Command::COMMAND_READ_PARAMS:
    rosflight_cmd = ROSFLIGHT_CMD_READ_PARAMS;
    break;
  case CommLink::Command::COMMAND_WRITE_PARAMS:
    rosflight_cmd = ROSFLIGHT_CMD_WRITE_PARAMS;
    break;
  case CommLink::Command::COMMAND_SET_PARAM_DEFAULTS:
    rosflight_cmd = ROSFLIGHT_CMD_SET_PARAM_DEFAULTS;
    break;
  case CommLink::Command::COMMAND_ACCEL_CALIBRATION:
    rosflight_cmd = ROSFLIGHT_CMD_ACCEL_CALIBRATION;
    break;
  case CommLink::Command::COMMAND_GYRO_CALIBRATION:
    rosflight_cmd = ROSFLIGHT_CMD_GYRO_CALIBRATION;
    break;
  case CommLink::Command::COMMAND_BARO_CALIBRATION:
    rosflight_cmd = ROSFLIGHT_CMD_BARO_CALIBRATION;
    break;
  case CommLink::Command::COMMAND_AIRSPEED_CALIBRATION:
    rosflight_cmd = ROSFLIGHT_CMD_AIRSPEED_CALIBRATION;
    break;
  case CommLink::Command::COMMAND_RC_CALIBRATION:
    rosflight_cmd = ROSFLIGHT_CMD_RC_CALIBRATION;
    break;
  case CommLink::Command::COMMAND_REBOOT:
    rosflight_cmd = ROSFLIGHT_CMD_REBOOT;
    break;
  case CommLink::Command::COMMAND_REBOOT_TO_BOOTLOADER:
    rosflight_cmd = ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER;
    break;
  case CommLink::Command::COMMAND_SEND_VERSION:
    rosflight_cmd = ROSFLIGHT_CMD_SEND_VERSION;
    break;
  }

  mavlink_message_t msg;
  mavlink_msg_rosflight_cmd_ack_pack(system_id, compid_, &msg,
                                     rosflight_cmd, (success) ? ROSFLIGHT_CMD_SUCCESS : ROSFLIGHT_CMD_FAILED);
  send_message(msg);
}

void Mavlink::send_diff_pressure(uint8_t system_id, float velocity, float pressure, float temperature)
{
  mavlink_message_t msg;
  mavlink_msg_diff_pressure_pack(system_id, compid_, &msg, velocity, pressure, temperature);
  send_message(msg);
}

void Mavlink::send_heartbeat(uint8_t system_id, bool fixed_wing)
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(system_id, compid_, &msg,
                             fixed_wing ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
                             0, 0, 0, 0);
  send_message(msg);
}

void Mavlink::send_imu(uint8_t system_id, uint64_t timestamp_us,
                       const turbomath::Vector &accel,
                       const turbomath::Vector &gyro,
                       float temperature)
{
  mavlink_message_t msg;
  mavlink_msg_small_imu_pack(system_id, compid_, &msg,
                             timestamp_us,
                             accel.x,
                             accel.y,
                             accel.z,
                             gyro.x,
                             gyro.y,
                             gyro.z,
                             temperature);
  send_message(msg);
}

void Mavlink::send_log_message(uint8_t system_id, LogSeverity severity, const char * text)
{
  MAV_SEVERITY mavlink_severity = MAV_SEVERITY_ENUM_END;
  switch (severity)
  {
  case CommLink::LogSeverity::LOG_INFO:
    mavlink_severity = MAV_SEVERITY_INFO;
    break;
  case CommLink::LogSeverity::LOG_WARNING:
    mavlink_severity = MAV_SEVERITY_WARNING;
    break;
  case CommLink::LogSeverity::LOG_ERROR:
    mavlink_severity = MAV_SEVERITY_ERROR;
    break;
  case CommLink::LogSeverity::LOG_CRITICAL:
    mavlink_severity = MAV_SEVERITY_CRITICAL;
    break;
  }

  mavlink_message_t msg;
  mavlink_msg_statustext_pack(system_id, compid_, &msg, static_cast<uint8_t>(mavlink_severity), text);
  send_message(msg);
}

void Mavlink::send_mag(uint8_t system_id, const turbomath::Vector &mag)
{
  mavlink_message_t msg;
  mavlink_msg_small_mag_pack(system_id, compid_, &msg, mag.x, mag.y, mag.z);
  send_message(msg);
}

void Mavlink::send_named_value_int(uint8_t system_id, uint32_t timestamp_ms, const char * const name, int32_t value)
{
  mavlink_message_t msg;
  mavlink_msg_named_value_int_pack(system_id, compid_, &msg, timestamp_ms, name, value);
  send_message(msg);
}

void Mavlink::send_named_value_float(uint8_t system_id, uint32_t timestamp_ms, const char * const name, float value)
{
  mavlink_message_t msg;
  mavlink_msg_named_value_float_pack(system_id, compid_, &msg, timestamp_ms, name, value);
  send_message(msg);
}

void Mavlink::send_output_raw(uint8_t system_id, uint32_t timestamp_ms, const float raw_outputs[8])
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_output_raw_pack(system_id, compid_, &msg, timestamp_ms, raw_outputs);
  send_message(msg);
}

void Mavlink::send_param_value_int(uint8_t system_id,
                                   uint16_t index,
                                   const char *const name,
                                   int32_t value,
                                   uint16_t param_count)
{
  mavlink_param_union_t param;
  param.param_int32 = value;

  mavlink_message_t msg;
  mavlink_msg_param_value_pack(system_id, 0, &msg, name, param.param_float, MAV_PARAM_TYPE_INT32, param_count, index);
  send_message(msg);
}

void Mavlink::send_param_value_float(uint8_t system_id,
                                     uint16_t index,
                                     const char *const name,
                                     float value,
                                     uint16_t param_count)
{
  mavlink_message_t msg;
  mavlink_msg_param_value_pack(system_id, 0, &msg, name, value, MAV_PARAM_TYPE_REAL32, param_count, index);
  send_message(msg);
}

void Mavlink::send_rc_raw(uint8_t system_id, uint32_t timestamp_ms, const uint16_t channels[8])
{
  mavlink_message_t msg;
  mavlink_msg_rc_channels_pack(system_id, compid_, &msg,
                               timestamp_ms,
                               0,
                               channels[0],
                               channels[1],
                               channels[2],
                               channels[3],
                               channels[4],
                               channels[5],
                               channels[6],
                               channels[7],
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  send_message(msg);
}

void Mavlink::send_sonar(uint8_t system_id, /* TODO enum type*/uint8_t type, float range, float max_range, float min_range)
{
  (void) type;
  mavlink_message_t msg;
  mavlink_msg_small_range_pack(system_id, compid_, &msg, /* TODO */ROSFLIGHT_RANGE_SONAR, range, max_range, min_range);
  send_message(msg);
}

void Mavlink::send_status(uint8_t system_id,
                 bool armed,
                 bool failsafe,
                 bool rc_override,
                 bool offboard,
                 uint8_t error_code,
                 uint8_t control_mode,
                 int16_t num_errors,
                 int16_t loop_time_us)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_status_pack(system_id, compid_, &msg,
                                    armed,
                                    failsafe,
                                    rc_override,
                                    offboard,
                                    error_code,
                                    control_mode,
                                    num_errors,
                                    loop_time_us);
  send_message(msg);
}

void Mavlink::send_timesync(uint8_t system_id, int64_t tc1, int64_t ts1)
{
  mavlink_message_t msg;
  mavlink_msg_timesync_pack(system_id, compid_, &msg, tc1, ts1);
  send_message(msg);
}

void Mavlink::send_version(uint8_t system_id, const char * const version)
{
  mavlink_message_t msg;
  mavlink_msg_rosflight_version_pack(system_id, compid_, &msg, version);
  send_message(msg);
}

void Mavlink::send_message(const mavlink_message_t &msg)
{
  if (initialized_)
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(data, &msg);
    board_.serial_write(data, len);
  }
}

void Mavlink::handle_msg_param_request_list(const mavlink_message_t *const msg)
{
  mavlink_param_request_list_t list;
  mavlink_msg_param_request_list_decode(msg, &list);
  param_request_list_callback_(list.target_system);
}

void Mavlink::handle_msg_param_request_read(const mavlink_message_t *const msg)
{
  mavlink_param_request_read_t read;
  mavlink_msg_param_request_read_decode(msg, &read);
  param_request_read_callback_(read.target_system, read.param_id, read.param_index);
}

void Mavlink::handle_msg_param_set(const mavlink_message_t *const msg)
{
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  mavlink_param_union_t param;
  param.param_float = set.param_value;
  param.type = set.param_type;

  switch (param.type)
  {
  case MAV_PARAM_TYPE_INT32:
    param_set_int_callback_(set.target_system, set.param_id, param.param_int32);
    break;
  case MAV_PARAM_TYPE_REAL32:
    param_set_float_callback_(set.target_system, set.param_id, param.param_float);
    break;
  default:
    // unsupported parameter type
    break;
  }
}

void Mavlink::handle_msg_rosflight_cmd(const mavlink_message_t *const msg)
{
  mavlink_rosflight_cmd_t cmd;
  mavlink_msg_rosflight_cmd_decode(msg, &cmd);

  CommLink::Command command;
  switch (cmd.command)
  {
  case ROSFLIGHT_CMD_READ_PARAMS:
    command = CommLink::Command::COMMAND_READ_PARAMS;
    break;
  case ROSFLIGHT_CMD_WRITE_PARAMS:
    command = CommLink::Command::COMMAND_WRITE_PARAMS;
    break;
  case ROSFLIGHT_CMD_SET_PARAM_DEFAULTS:
    command = CommLink::Command::COMMAND_SET_PARAM_DEFAULTS;
    break;
  case ROSFLIGHT_CMD_ACCEL_CALIBRATION:
    command = CommLink::Command::COMMAND_ACCEL_CALIBRATION;
    break;
  case ROSFLIGHT_CMD_GYRO_CALIBRATION:
    command = CommLink::Command::COMMAND_GYRO_CALIBRATION;
    break;
  case ROSFLIGHT_CMD_BARO_CALIBRATION:
    command = CommLink::Command::COMMAND_BARO_CALIBRATION;
    break;
  case ROSFLIGHT_CMD_AIRSPEED_CALIBRATION:
    command = CommLink::Command::COMMAND_AIRSPEED_CALIBRATION;
    break;
  case ROSFLIGHT_CMD_RC_CALIBRATION:
    command = CommLink::Command::COMMAND_RC_CALIBRATION;
    break;
  case ROSFLIGHT_CMD_REBOOT:
    command = CommLink::Command::COMMAND_REBOOT;
    break;
  case ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER:
    command = CommLink::Command::COMMAND_REBOOT_TO_BOOTLOADER;
    break;
  case ROSFLIGHT_CMD_SEND_VERSION:
    command = CommLink::Command::COMMAND_SEND_VERSION;
    break;
  default: // unsupported command; report failure then return without calling command callback
    mavlink_message_t out_msg;
    mavlink_msg_rosflight_cmd_ack_pack(msg->sysid, compid_, &out_msg, cmd.command, ROSFLIGHT_CMD_FAILED);
    send_message(out_msg);
    // log(LogSeverity::LOG_ERROR, "Unsupported ROSFLIGHT CMD %d", command);
    return;
  }

  command_callback_(command);
}

void Mavlink::handle_msg_timesync(const mavlink_message_t *const msg)
{
  mavlink_timesync_t tsync;
  mavlink_msg_timesync_decode(msg, &tsync);
  timesync_callback_(tsync.tc1, tsync.ts1);
}

void Mavlink::handle_msg_offboard_control(const mavlink_message_t *const msg)
{
  mavlink_offboard_control_t ctrl;
  mavlink_msg_offboard_control_decode(msg, &ctrl);

  CommLink::OffboardControl control;
  switch (ctrl.mode)
  {
  case MODE_PASS_THROUGH:
    control.mode = CommLink::OffboardControl::Mode::PASS_THROUGH;
    break;
  case MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    control.mode = CommLink::OffboardControl::Mode::ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    break;
  case MODE_ROLL_PITCH_YAWRATE_THROTTLE:
    control.mode = CommLink::OffboardControl::Mode::ROLL_PITCH_YAWRATE_THROTTLE;
    break;
  default:
    // invalid mode; ignore message and return without calling callback
    return;
  }

  control.x.value = ctrl.x;
  control.y.value = ctrl.y;
  control.z.value = ctrl.z;
  control.F.value = ctrl.F;

  control.x.valid = !(ctrl.ignore & IGNORE_VALUE1);
  control.y.valid = !(ctrl.ignore & IGNORE_VALUE2);
  control.z.valid = !(ctrl.ignore & IGNORE_VALUE3);
  control.F.valid = !(ctrl.ignore & IGNORE_VALUE4);

  offboard_control_callback_(control);
}

void Mavlink::handle_mavlink_message(void)
{
  switch (in_buf_.msgid)
  {
  case MAVLINK_MSG_ID_OFFBOARD_CONTROL:
    handle_msg_offboard_control(&in_buf_);
    break;
  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    handle_msg_param_request_list(&in_buf_);
    break;
  case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    handle_msg_param_request_read(&in_buf_);
    break;
  case MAVLINK_MSG_ID_PARAM_SET:
    handle_msg_param_set(&in_buf_);
    break;
  case MAVLINK_MSG_ID_ROSFLIGHT_CMD:
    handle_msg_rosflight_cmd(&in_buf_);
    break;
  case MAVLINK_MSG_ID_TIMESYNC:
    handle_msg_timesync(&in_buf_);
    break;
  default:
    break;
  }
}

} // namespace rosflight_firmware
