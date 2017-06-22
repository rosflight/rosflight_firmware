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
#include <stdint.h>

#include "mavlink.h"
#include "rosflight.h"

namespace rosflight_firmware {

Mavlink::Mavlink()
{
  initialized = false;
}

// function definitions
void Mavlink::init(ROSflight *firmware)
{
  RF_ = firmware;
  RF_->board_->serial_init(RF_->params_.get_param_int(PARAM_BAUD_RATE));

  sysid = RF_->params_.get_param_int(PARAM_SYSTEM_ID);
  compid = 250;

  _offboard_control_time = 0;
  send_params_index = PARAMS_COUNT;

  // Register Param change callbacks
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_HEARTBEAT, std::placeholders::_1), PARAM_STREAM_HEARTBEAT_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_ATTITUDE, std::placeholders::_1), PARAM_STREAM_ATTITUDE_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_IMU, std::placeholders::_1), PARAM_STREAM_IMU_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_ATTITUDE, std::placeholders::_1), PARAM_STREAM_ATTITUDE_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_DIFF_PRESSURE, std::placeholders::_1), PARAM_STREAM_AIRSPEED_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_BARO, std::placeholders::_1), PARAM_STREAM_BARO_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_SONAR, std::placeholders::_1), PARAM_STREAM_SONAR_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_MAG, std::placeholders::_1), PARAM_STREAM_MAG_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_SERVO_OUTPUT_RAW, std::placeholders::_1), PARAM_STREAM_OUTPUT_RAW_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_RC_RAW, std::placeholders::_1), PARAM_STREAM_RC_RAW_RATE);

  initialized = true;
}

void Mavlink::send_message(const mavlink_message_t &msg)
{
  if (initialized)
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(data, &msg);
    for (int i = 0; i < len; i++)
    {
      RF_->board_->serial_write(data[i]);
    }
  }
}

void Mavlink::update_status()
{
  mavlink_send_status();
}


void Mavlink::update_param(uint16_t param_id)
{
  if (param_id < PARAMS_COUNT)
  {
    MAV_PARAM_TYPE type;
    switch (RF_->params_.get_param_type(param_id))
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

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                                 RF_->params_.get_param_name(param_id), RF_->params_.get_param_float(param_id), type, PARAMS_COUNT, param_id);
    send_message(msg);
  }
}

void Mavlink::mavlink_handle_msg_param_request_list(void)
{
  send_params_index = 0;
}

void Mavlink::mavlink_handle_msg_param_request_read(const mavlink_message_t *const msg)
{
  mavlink_param_request_read_t read;
  mavlink_msg_param_request_read_decode(msg, &read);

  if (read.target_system == (uint8_t) RF_->params_.get_param_int(PARAM_SYSTEM_ID)) // TODO check if component id matches?
  {
    uint16_t id = (read.param_index < 0) ? RF_->params_.lookup_param_id(read.param_id) : (uint16_t) read.param_index;

    if (id < PARAMS_COUNT)
      update_param(id);
  }
}

void Mavlink::mavlink_handle_msg_param_set(const mavlink_message_t *const msg)
{
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  if (set.target_system == (uint8_t) RF_->params_.get_param_int(PARAM_SYSTEM_ID)) // TODO check if component id matches?
  {
    uint16_t id = RF_->params_.lookup_param_id(set.param_id);

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

      if (candidate_type == RF_->params_.get_param_type(id))
      {
        switch (candidate_type)
        {
        case PARAM_TYPE_INT32:
          RF_->params_.set_param_int(id, *(int32_t *) &set.param_value);
          break;
        case PARAM_TYPE_FLOAT:
          RF_->params_.set_param_float(id, set.param_value);
          break;
        }
      }
    }
  }
}

void Mavlink::mavlink_send_next_param(void)
{
  if (send_params_index < PARAMS_COUNT)
  {
    update_param((uint16_t) send_params_index);
    send_params_index++;
  }
}


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
      RF_->board_->baro_calibrate();
      break;
    case ROSFLIGHT_CMD_AIRSPEED_CALIBRATION:
      RF_->board_->diff_pressure_calibrate();
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
      log_error(this, "unsupported ROSFLIGHT CMD %d", cmd.command);
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
    RF_->board_->clock_delay(20);
    RF_->board_->board_reset(reboot_to_bootloader_flag);
  }
}

void Mavlink::mavlink_handle_msg_timesync(const mavlink_message_t *const msg)
{
  uint64_t now_us = RF_->board_->clock_micros();

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
  _offboard_control_time = RF_->board_->clock_micros();
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
  while (RF_->board_->serial_bytes_available())
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, RF_->board_->serial_read(), &in_buf, &status))
      handle_mavlink_message();
  }
}

void Mavlink::send_log_message(uint8_t severity, char *text)
{
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                              severity,
                              text);
  send_message(msg);
}


void Mavlink::mavlink_send_heartbeat(void)
{
    uint8_t control_mode = 0;
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                             RF_->params_.get_param_int(PARAM_FIXED_WING) ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
                             0, 0, 0, 0);
  send_message(msg);
}

void Mavlink::mavlink_send_status(void)
{
  if (!initialized)
    return;
  volatile uint8_t status = 0;
  status |= (RF_->state_manager_.state().armed) ? ROSFLIGHT_STATUS_ARMED : 0x00;
  status |= (RF_->state_manager_.state().failsafe) ? ROSFLIGHT_STATUS_IN_FAILSAFE : 0x00;
  status |= (RF_->mux_.rc_override_active()) ? ROSFLIGHT_STATUS_RC_OVERRIDE : 0x00;
  status |= (RF_->mux_.offboard_control_active()) ? ROSFLIGHT_STATUS_OFFBOARD_CONTROL_ACTIVE : 0x00;

  uint8_t control_mode = 0;
  if (RF_->params_.get_param_int(PARAM_FIXED_WING))
    control_mode = MODE_PASS_THROUGH;
  else if (RF_->mux_._combined_control.x.type == ANGLE)
    control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  else
    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;

  mavlink_message_t msg;
  mavlink_msg_rosflight_status_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                                    status,
                                    RF_->state_manager_.state().error_codes,
                                    control_mode,
                                    RF_->board_->num_sensor_errors(),
                                    RF_->get_loop_time_us());
  send_message(msg);
}


void Mavlink::mavlink_send_attitude(void)
{
  mavlink_message_t msg;
  mavlink_msg_attitude_quaternion_pack(sysid, compid, &msg,
                                       RF_->estimator_.get_estimator_timestamp()/1000,
                                       RF_->estimator_.get_attitude().w,
                                       RF_->estimator_.get_attitude().x,
                                       RF_->estimator_.get_attitude().y,
                                       RF_->estimator_.get_attitude().z,
                                       RF_->estimator_.get_angular_velocity().x,
                                       RF_->estimator_.get_angular_velocity().y,
                                       RF_->estimator_.get_angular_velocity().z);
  send_message(msg);
}

void Mavlink::mavlink_send_imu(void)
{
  if(RF_->sensors_.should_send_imu_data())
  {
    mavlink_message_t msg;
    vector_t accel = RF_->sensors_.data()._accel;
    vector_t gyro = RF_->sensors_.data()._gyro;
    mavlink_msg_small_imu_pack(sysid, compid, &msg,
                               RF_->sensors_.data()._imu_time,
                               accel.x,
                               accel.y,
                               accel.z,
                               gyro.x,
                               gyro.y,
                               gyro.z,
                               RF_->sensors_.data()._imu_temperature);
    send_message(msg);
  }
  else
  {
    // Otherwise, wait and signal that we still need to send IMU
    mavlink_streams[STREAM_ID_IMU].next_time_us -= mavlink_streams[STREAM_ID_IMU].period_us;
  }

}

void Mavlink::mavlink_send_output_raw(void)
{
  mavlink_message_t msg;
    mavlink_msg_rosflight_output_raw_pack(sysid, compid, &msg,
                                      RF_->board_->clock_millis(),
                                      RF_->mixer_._outputs);
    send_message(msg);
}

void Mavlink::mavlink_send_rc_raw(void)
{
  mavlink_message_t msg;
  mavlink_msg_rc_channels_pack(sysid, compid, &msg,
                               RF_->board_->clock_millis(),
                               0,
                               RF_->board_->pwm_read(0),
                               RF_->board_->pwm_read(1),
                               RF_->board_->pwm_read(2),
                               RF_->board_->pwm_read(3),
                               RF_->board_->pwm_read(4),
                               RF_->board_->pwm_read(5),
                               RF_->board_->pwm_read(6),
                               RF_->board_->pwm_read(7),
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0);
  send_message(msg);
}

void Mavlink::mavlink_send_diff_pressure(void)
{
  if (RF_->board_->diff_pressure_present())
  {
    mavlink_message_t msg;
    mavlink_msg_diff_pressure_pack(sysid, compid, &msg,
                                   RF_->sensors_.data()._diff_pressure_velocity,
                                   RF_->sensors_.data()._diff_pressure,
                                   RF_->sensors_.data()._diff_pressure_temp);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_baro(void)
{
  if (RF_->board_->baro_present())
  {
    mavlink_message_t msg;
    mavlink_msg_small_baro_pack(sysid, compid, &msg,
                                RF_->sensors_.data()._baro_altitude,
                                RF_->sensors_.data()._baro_pressure,
                                RF_->sensors_.data()._baro_temperature);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_sonar(void)
{
  if (RF_->board_->sonar_present())
  {
    mavlink_message_t msg;
    mavlink_msg_small_range_pack(sysid, compid, &msg,
                                 ROSFLIGHT_RANGE_SONAR,
                                 RF_->sensors_.data()._sonar_range,
                                 8.0,
                                 0.25);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_mag(void)
{
  if (RF_->board_->mag_present())
  {
    mavlink_message_t msg;
    mavlink_msg_small_mag_pack(sysid, compid, &msg,
                               RF_->sensors_.data()._mag.x,
                               RF_->sensors_.data()._mag.y,
                               RF_->sensors_.data()._mag.z);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}

// function definitions
void Mavlink::stream()
{
  uint64_t time_us = RF_->board_->clock_micros();
  for (int i = 0; i < STREAM_COUNT; i++)
  {
    if (time_us >= mavlink_streams[i].next_time_us)
    {
      // if we took too long, set the last_time_us to be where it should have been
      mavlink_streams[i].next_time_us += mavlink_streams[i].period_us;
      (this->*mavlink_streams[i].send_function)();
    }
  }
}

void Mavlink::set_streaming_rate(uint8_t stream_id, int16_t param_id)
{
  mavlink_streams[stream_id].period_us = (RF_->params_.get_param_int(param_id) == 0 ? 0 : 1000000/RF_->params_.get_param_int(param_id));
}

void Mavlink::mavlink_stream_set_period(uint8_t stream_id, uint32_t period_us)
{
  mavlink_streams[stream_id].period_us = period_us;
}


void Mavlink::mavlink_send_named_value_int(const char *const name, int32_t value)
{
  mavlink_message_t msg;
  mavlink_msg_named_value_int_pack(sysid, compid, &msg, RF_->board_->clock_millis(), name, value);
  send_message(msg);
}

void Mavlink::mavlink_send_named_value_float(const char *const name, float value)
{
  mavlink_message_t msg;
  mavlink_msg_named_value_float_pack(sysid, compid, &msg, RF_->board_->clock_millis(), name, value);
  send_message(msg);
}

//void Mavlink::mavlink_send_named_command_struct(const char *const name, control_t command_struct)
//{
//  uint8_t control_mode;
//  if (command_struct.x.type == RATE && command_struct.y.type == RATE)
//  {
//    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
//  }
//  else if (command_struct.x.type == ANGLE && command_struct.y.type == ANGLE)
//  {
//    if (command_struct.x.type == ALTITUDE)
//    {
//      control_mode = MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
//    }
//    else
//    {
//      control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
//    }
//  }
//  else
//  {
//    control_mode = MODE_PASS_THROUGH;
//  }
//  uint8_t ignore = !(command_struct.x.active) ||
//                   !(command_struct.y.active) << 1 ||
//                   !(command_struct.z.active) << 2 ||
//                   !(command_struct.F.active) << 3;
//  mavlink_message_t msg;
//  mavlink_msg_named_command_struct_pack(sysid, compid, &msg, name,
//                                        control_mode,
//                                        ignore,
//                                        command_struct.x.value,
//                                        command_struct.y.value,
//                                        command_struct.z.value,
//                                        command_struct.F.value);
//  send_message(msg);
//}


}

