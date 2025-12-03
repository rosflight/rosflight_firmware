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

#include "comm_manager.h"

#include "param.h"

#include "rosflight.h"

#include <cstdint>
#include <cstring>

namespace rosflight_firmware
{
CommManager::LogMessageBuffer::LogMessageBuffer() { memset(buffer_, 0, sizeof(buffer_)); }

void CommManager::LogMessageBuffer::add_message(CommLinkInterface::LogSeverity severity, char msg[])
{
  LogMessage & newest_msg = buffer_[newest_];
  strcpy(newest_msg.msg, msg);
  newest_msg.severity = severity;

  newest_ = (newest_ + 1) % LOG_BUF_SIZE;

  // quietly over-write old messages (what else can we do?)
  length_ += 1;
  if (length_ > LOG_BUF_SIZE) {
    length_ = LOG_BUF_SIZE;
    oldest_ = (oldest_ + 1) % LOG_BUF_SIZE;
  }
}

void CommManager::LogMessageBuffer::pop()
{
  if (length_ > 0) {
    length_--;
    oldest_ = (oldest_ + 1) % LOG_BUF_SIZE;
  }
}

CommManager::CommManager(ROSflight & rf, CommLinkInterface & comm_link)
    : RF_(rf)
    , comm_link_(comm_link)
{}

// function definitions
void CommManager::init()
{
  comm_link_.init(static_cast<uint32_t>(RF_.params_.get_param_int(PARAM_BAUD_RATE)),
                  static_cast<uint32_t>(RF_.params_.get_param_int(PARAM_SERIAL_DEVICE)));

  offboard_control_time_ = 0;
  send_params_index_ = PARAMS_COUNT;

  update_system_id(PARAM_SYSTEM_ID);

  initialized_ = true;
}

void CommManager::param_change_callback(uint16_t param_id)
{
  switch (param_id) {
    case PARAM_SYSTEM_ID:
      update_system_id(param_id);
      break;
    default:
      // do nothing
      break;
  }
}

void CommManager::update_system_id(uint16_t param_id)
{
  sysid_ = static_cast<uint8_t>(RF_.params_.get_param_int(param_id));
}

void CommManager::update_status() { send_status(); }

void CommManager::send_param_value(uint16_t param_id)
{
  if (param_id < PARAMS_COUNT) {
    switch (RF_.params_.get_param_type(param_id)) {
      case PARAM_TYPE_INT32:
        comm_link_.send_param_value_int(sysid_, param_id, RF_.params_.get_param_name(param_id),
                                        RF_.params_.get_param_int(param_id),
                                        static_cast<uint16_t>(PARAMS_COUNT));
        break;
      case PARAM_TYPE_FLOAT:
        comm_link_.send_param_value_float(sysid_, param_id, RF_.params_.get_param_name(param_id),
                                          RF_.params_.get_param_float(param_id),
                                          static_cast<uint16_t>(PARAMS_COUNT));
        break;
      default:
        break;
    }
  }
}

void CommManager::param_request_list_callback(uint8_t target_system)
{
  if (target_system == sysid_) { send_params_index_ = 0; }
}

void CommManager::param_request_read_callback(uint8_t target_system, const char * const param_name,
                                              int16_t param_index)
{
  if (target_system == sysid_) {
    uint16_t id = (param_index < 0) ? RF_.params_.lookup_param_id(param_name)
                                    : static_cast<uint16_t>(param_index);

    if (id < PARAMS_COUNT) { send_param_value(id); }
  }
}

void CommManager::param_set_int_callback(uint8_t target_system, const char * const param_name,
                                         int32_t param_value)
{
  if (target_system == sysid_) {
    uint16_t id = RF_.params_.lookup_param_id(param_name);

    if (id < PARAMS_COUNT && RF_.params_.get_param_type(id) == PARAM_TYPE_INT32) {
      RF_.params_.set_param_int(id, param_value);
    }
  }
}

void CommManager::param_set_float_callback(uint8_t target_system, const char * const param_name,
                                           float param_value)
{
  if (target_system == sysid_) {
    uint16_t id = RF_.params_.lookup_param_id(param_name);

    if (id < PARAMS_COUNT && RF_.params_.get_param_type(id) == PARAM_TYPE_FLOAT) {
      RF_.params_.set_param_float(id, param_value);
    }
  }
}

void CommManager::command_callback(CommLinkInterface::Command command)
{
  bool result;
  bool reboot_flag = false;
  bool reboot_to_bootloader_flag = false;

  // None of these actions can be performed if we are armed
  if (RF_.state_manager_.state().armed) {
    result = false;
  } else {
    result = true;
    switch (command) {
      case CommLinkInterface::Command::COMMAND_READ_PARAMS:
        result = RF_.params_.read();
        break;
      case CommLinkInterface::Command::COMMAND_WRITE_PARAMS:
        result = RF_.params_.write();
        break;
      case CommLinkInterface::Command::COMMAND_SET_PARAM_DEFAULTS:
        RF_.params_.set_defaults();
        break;
      case CommLinkInterface::Command::COMMAND_ACCEL_CALIBRATION:
        result = RF_.sensors_.start_imu_calibration();
        break;
      case CommLinkInterface::Command::COMMAND_GYRO_CALIBRATION:
        result = RF_.sensors_.start_gyro_calibration();
        break;
      case CommLinkInterface::Command::COMMAND_BARO_CALIBRATION:
        result = RF_.sensors_.start_baro_calibration();
        break;
      case CommLinkInterface::Command::COMMAND_AIRSPEED_CALIBRATION:
        result = RF_.sensors_.start_diff_pressure_calibration();
        break;
      case CommLinkInterface::Command::COMMAND_RC_CALIBRATION:
        RF_.controller_.calculate_equilbrium_torque_from_rc();
        break;
      case CommLinkInterface::Command::COMMAND_REBOOT:
        reboot_flag = true;
        break;
      case CommLinkInterface::Command::COMMAND_REBOOT_TO_BOOTLOADER:
        reboot_to_bootloader_flag = true;
        break;
      case CommLinkInterface::Command::COMMAND_SEND_VERSION:
        comm_link_.send_version(sysid_, GIT_VERSION_STRING);
        break;
    }
  }

  comm_link_.send_command_ack(sysid_, command, result);

  if (reboot_flag || reboot_to_bootloader_flag) {
    RF_.board_.clock_delay(20);
    RF_.board_.board_reset(reboot_to_bootloader_flag);
  }
  RF_.board_.serial_flush();
}

void CommManager::timesync_callback(int64_t tc1, int64_t ts1)
{
  if (tc1 == 0) {
    // check that this is a request, not a response
    tc1 = RF_.board_.clock_micros()*1000L;
    comm_link_.send_timesync(sysid_, tc1, ts1);
  }
}

void CommManager::offboard_control_callback(const CommLinkInterface::OffboardControl & control)
{
  // put values into a new command struct
  control_t new_offboard_command;
  new_offboard_command.u[0].value = control.u[0].value;
  new_offboard_command.u[1].value = control.u[1].value;
  new_offboard_command.u[2].value = control.u[2].value;
  new_offboard_command.u[3].value = control.u[3].value;
  new_offboard_command.u[4].value = control.u[4].value;
  new_offboard_command.u[5].value = control.u[5].value;

  // Move flags into standard message
  new_offboard_command.u[0].active = control.u[0].valid;
  new_offboard_command.u[1].active = control.u[1].valid;
  new_offboard_command.u[2].active = control.u[2].valid;
  new_offboard_command.u[3].active = control.u[3].valid;
  new_offboard_command.u[4].active = control.u[4].valid;
  new_offboard_command.u[5].active = control.u[5].valid;

  // translate modes into standard message
  switch (control.mode) {
    case CommLinkInterface::OffboardControl::Mode::PASS_THROUGH:
      new_offboard_command.u[0].type = PASSTHROUGH;
      new_offboard_command.u[1].type = PASSTHROUGH;
      new_offboard_command.u[2].type = PASSTHROUGH;
      new_offboard_command.u[3].type = PASSTHROUGH;
      new_offboard_command.u[4].type = PASSTHROUGH;
      new_offboard_command.u[5].type = PASSTHROUGH;
      break;
    case CommLinkInterface::OffboardControl::Mode::ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
      new_offboard_command.u[0].type = RATE;
      new_offboard_command.u[1].type = RATE;
      new_offboard_command.u[2].type = RATE;
      new_offboard_command.u[3].type = THROTTLE;
      new_offboard_command.u[4].type = THROTTLE;
      new_offboard_command.u[5].type = THROTTLE;
      break;
    case CommLinkInterface::OffboardControl::Mode::ROLL_PITCH_YAWRATE_THROTTLE:
      new_offboard_command.u[0].type = ANGLE;
      new_offboard_command.u[1].type = ANGLE;
      new_offboard_command.u[2].type = RATE;
      new_offboard_command.u[3].type = THROTTLE;
      new_offboard_command.u[4].type = THROTTLE;
      new_offboard_command.u[5].type = THROTTLE;
      break;
  }

  // Tell the command_manager that we have a new command we need to mux
  new_offboard_command.stamp_ms = RF_.board_.clock_millis();
  RF_.command_manager_.set_new_offboard_command(new_offboard_command);
}

void CommManager::aux_command_callback(const CommLinkInterface::AuxCommand & command)
{
  Mixer::aux_command_t new_aux_command;

  for (int i = 0; i < 14; i++) {
    switch (command.cmd_array[i].type) {
      case CommLinkInterface::AuxCommand::Type::DISABLED:
        // Channel is either not used or is controlled by the mixer
        new_aux_command.channel[i].type = Mixer::AUX;
        new_aux_command.channel[i].value = 0;
        break;
      case CommLinkInterface::AuxCommand::Type::SERVO:
        // PWM value should be mapped to servo position
        new_aux_command.channel[i].type = Mixer::S;
        new_aux_command.channel[i].value = command.cmd_array[i].value;
        break;
      case CommLinkInterface::AuxCommand::Type::MOTOR:
        // PWM value should be mapped to motor speed
        new_aux_command.channel[i].type = Mixer::M;
        new_aux_command.channel[i].value = command.cmd_array[i].value;
        break;
    }
  }

  // Send the new aux_command to the mixer
  RF_.mixer_.set_new_aux_command(new_aux_command);
}

void CommManager::external_attitude_callback(const turbomath::Quaternion & q)
{
  RF_.estimator_.set_external_attitude_update(q);
}

void CommManager::heartbeat_callback(void)
{
  // receiving a heartbeat implies that a connection has been made
  // to the off-board computer.
  connected_ = true;

  // send backup data if we have it buffered
  if (have_backup_data_) {
    comm_link_.send_error_data(sysid_, backup_data_buffer_);
    have_backup_data_ = false;
  }
}

// function definitions
void CommManager::receive(void) { comm_link_.receive(); }

void CommManager::log(CommLinkInterface::LogSeverity severity, const char * fmt, ...)
{
  // Convert the format string to a raw char array
  va_list args;
  va_start(args, fmt);
  char message[LOG_MSG_SIZE];
  vsnprintf(message, LOG_MSG_SIZE, fmt, args);
  va_end(args);

  log_message(severity, message);
}

void CommManager::log_message(CommLinkInterface::LogSeverity severity, char * text)
{
  if (initialized_ && connected_) {
    comm_link_.send_log_message(sysid_, severity, text);
  } else {
    log_buffer_.add_message(severity, text);
  }
}

void CommManager::send_heartbeat(void)
{
  comm_link_.send_heartbeat(sysid_, static_cast<bool>(RF_.params_.get_param_int(PARAM_FIXED_WING)));
}

void CommManager::send_status(void)
{
  if (!initialized_) { return; }

  uint8_t control_mode = 0;
  if (RF_.params_.get_param_int(PARAM_FIXED_WING)
      || RF_.command_manager_.combined_control().u[0].type == PASSTHROUGH) {
    control_mode = MODE_PASS_THROUGH;
  } else if (RF_.command_manager_.combined_control().u[0].type == ANGLE) {
    control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  } else {
    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  }

  comm_link_.send_status(
    sysid_, RF_.state_manager_.state().armed, RF_.state_manager_.state().failsafe,
    RF_.command_manager_.get_rc_override(), RF_.command_manager_.offboard_control_active(),
    RF_.state_manager_.state().error_codes, control_mode, RF_.board_.sensors_errors_count(),
    RF_.get_loop_time_us());
}

void CommManager::send_attitude(void)
{
  comm_link_.send_attitude_quaternion(sysid_, RF_.estimator_.state().timestamp_us,
                                      RF_.estimator_.state().attitude,
                                      RF_.estimator_.state().angular_velocity);
}

void CommManager::send_imu(void)
{
  turbomath::Vector acc, gyro;
  uint64_t stamp_us;
  RF_.sensors_.get_filtered_IMU(acc, gyro, stamp_us);
  comm_link_.send_imu(sysid_, stamp_us, acc, gyro, RF_.sensors_.get_imu()->temperature);
}

void CommManager::send_output_raw(void)
{
  comm_link_.send_output_raw(sysid_, RF_.board_.clock_millis(), RF_.mixer_.get_outputs());
}

void CommManager::send_rc_raw(void)
{
  rosflight_firmware::RcStruct * rc_struct =RF_.rc_.get_rc();

  size_t n = (sizeof(rc_struct->chan)<8) ? sizeof(rc_struct->chan):8;
  uint16_t channels[8];
  for(size_t i=0;i<n;i++) channels[i] = rc_struct->chan[i]*1000.0 + 1000;
  comm_link_.send_rc_raw(sysid_, RF_.board_.clock_millis(), channels);
}

void CommManager::send_diff_pressure(void)
{
  comm_link_.send_diff_pressure(sysid_, RF_.sensors_.get_diff_pressure()->ias,
                                RF_.sensors_.get_diff_pressure()->pressure,
                                RF_.sensors_.get_diff_pressure()->temperature);
}

void CommManager::send_baro(void)
{
  comm_link_.send_baro(sysid_, RF_.sensors_.get_baro()->altitude, RF_.sensors_.get_baro()->pressure,
      RF_.sensors_.get_baro()->temperature);
}

void CommManager::send_sonar(void)
{
  comm_link_.send_sonar(sysid_,
                        0, // TODO set sensor type (sonar/lidar), use enum
                        RF_.sensors_.get_sonar()->range, 8.0, 0.25);
}

void CommManager::send_mag(void)
{
  turbomath::Vector flux ( RF_.sensors_.get_mag()->flux[0], RF_.sensors_.get_mag()->flux[1], RF_.sensors_.get_mag()->flux[2]);
  comm_link_.send_mag(sysid_, flux );

}
void CommManager::send_battery_status(void)
{
  comm_link_.send_battery_status(sysid_, RF_.sensors_.get_battery()->voltage,
                                 RF_.sensors_.get_battery()->current);
}

void CommManager::send_backup_data(const StateManager::BackupData & backup_data)
{
  if (connected_) {
    comm_link_.send_error_data(sysid_, backup_data);
  } else {
    backup_data_buffer_ = backup_data;
    have_backup_data_ = true;
  }
}

void CommManager::send_gnss(void)
{
  comm_link_.send_gnss(sysid_, RF_.sensors_.get_gnss());
}

void CommManager::send_1hz_heartbeat(void)
{
  uint64_t time_us = RF_.board_.clock_micros();
  static uint64_t next_heartbeat = 0, next_status = 0;

  if ((time_us) / 1000000 >= next_heartbeat) { // 1 Hz
    send_heartbeat();
    next_heartbeat = time_us / 1000000 + 1;
  }
  if ((time_us) / 100000 >= next_status) { // 10 Hz
    send_status();
    next_status = time_us / 100000 + 1;
  }
}

void CommManager::send_buffered_log_messages(void)
{
  if (connected_ && !log_buffer_.empty()) {
    const LogMessageBuffer::LogMessage & msg = log_buffer_.oldest();
    comm_link_.send_log_message(RF_.board_.clock_micros(), msg.severity, msg.msg);
    log_buffer_.pop();
  }
}

// function definitions
void CommManager::stream(got_flags got)
{
  // Send out data

  if (got.imu) { // Nominally 400Hz
    send_imu();
    send_attitude();
    static uint64_t ro_count = 0;
    if (!((ro_count++) % 8)) { send_output_raw(); } // Raw output at 400Hz/8 = 50Hz
  }

  // Pitot sensor
  if (got.diff_pressure) { send_diff_pressure(); }
  // Baro altitude
  if (got.baro) { send_baro(); }
  // Magnetometer
  if (got.mag) { send_mag(); }
  // Height above ground sensor
  if (got.sonar) { send_sonar(); }
  // Battery V & I
  if (got.battery) { send_battery_status(); }
  // GPS data (GNSS Packed)
  if (got.gnss) { send_gnss(); }

  send_1hz_heartbeat();

  send_next_param();

  send_buffered_log_messages();

}

void CommManager::send_next_param(void)
{
  if (send_params_index_ < PARAMS_COUNT) {
    send_param_value(static_cast<uint16_t>(send_params_index_));
    send_params_index_++;
  }
}

CommManager::Stream::Stream(uint32_t period_us, std::function<void(void)> send_function)
    : period_us_(period_us)
    , next_time_us_(0)
    , send_function_(send_function)
{}

void CommManager::Stream::stream(uint64_t now_us)
{
  if (period_us_ > 0 && now_us >= next_time_us_) {
    // if you fall behind, skip messages
    do {
      next_time_us_ += period_us_;
    } while (next_time_us_ < now_us);

    send_function_();
  }
}

void CommManager::Stream::set_rate(uint32_t rate_hz)
{
  period_us_ = (rate_hz == 0) ? 0 : 1000000 / rate_hz;
}

} // namespace rosflight_firmware
