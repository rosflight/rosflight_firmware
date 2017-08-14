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

#ifndef ROSFLIGHT_FIRMWARE_COMM_MANAGER_H
#define ROSFLIGHT_FIRMWARE_COMM_MANAGER_H

#include <cstdint>
#include <functional>

#include "comm_link.h"
#include "nanoprintf.h"

namespace rosflight_firmware
{

class ROSflight;

class CommManager
{
private:
  enum StreamId
  {
    STREAM_ID_HEARTBEAT,
    STREAM_ID_STATUS,

    STREAM_ID_ATTITUDE,

    STREAM_ID_IMU,
    STREAM_ID_DIFF_PRESSURE,
    STREAM_ID_BARO,
    STREAM_ID_SONAR,
    STREAM_ID_MAG,

    STREAM_ID_SERVO_OUTPUT_RAW,
    STREAM_ID_RC_RAW,
    STREAM_ID_LOW_PRIORITY,
    STREAM_COUNT
  };

  enum OffboardControlMode
  {
    MODE_PASS_THROUGH,
    MODE_ROLL_PITCH_YAWRATE_THROTTLE,
    MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
  };

  uint8_t sysid_;
  uint64_t offboard_control_time_;
  ROSflight& RF_;
  CommLink& comm_link_;
  uint8_t send_params_index_;
  bool initialized_;

  typedef  void (CommManager::*CommManagerStreamFcn)(void);

  typedef struct
  {
    uint32_t period_us;
    uint64_t next_time_us;
    CommManagerStreamFcn send_function;
  } mavlink_stream_t;

  void update_system_id(uint16_t param_id);

  void param_request_list_callback(uint8_t target_system);
  void param_request_read_callback(uint8_t target_system, const char* const param_name, uint16_t param_index);
  void param_set_callback(uint8_t target_system, const char* const param_name, float param_value, param_type_t param_type);
  void rosflight_command_callback(uint8_t command);
  void timesync_callback(int64_t tc1, int64_t ts1);
  void offboard_control_callback(uint8_t mode, uint8_t ignore, float x, float y, float z, float F);

  void send_heartbeat(void);
  void send_status(void);
  void send_attitude(void);
  void send_imu(void);
  void send_output_raw(void);
  void send_rc_raw(void);
  void send_diff_pressure(void);
  void send_baro(void);
  void send_sonar(void);
  void send_mag(void);
  void send_low_priority(void);

  // Debugging Utils
  void send_named_value_int(const char *const name, int32_t value);
  //  void send_named_command_struct(const char *const name, control_t command_struct);

  void send_next_param(void);

  mavlink_stream_t mavlink_streams_[STREAM_COUNT] = {
    //  period_us    last_time_us   send_function
    { 1000000,     0,             &rosflight_firmware::CommManager::send_heartbeat },
    { 1000000,     0,             &rosflight_firmware::CommManager::send_status},
    { 200000,      0,             &rosflight_firmware::CommManager::send_attitude },
    { 1000,        0,             &rosflight_firmware::CommManager::send_imu },
    { 200000,      0,             &rosflight_firmware::CommManager::send_diff_pressure },
    { 200000,      0,             &rosflight_firmware::CommManager::send_baro },
    { 100000,      0,             &rosflight_firmware::CommManager::send_sonar },
    { 6250,        0,             &rosflight_firmware::CommManager::send_mag },
    { 0,           0,             &rosflight_firmware::CommManager::send_output_raw },
    { 0,           0,             &rosflight_firmware::CommManager::send_rc_raw },
    { 5000,        0,             &rosflight_firmware::CommManager::send_low_priority }
  };

public:

  enum
  {
    LOG_INFO = 6,
    LOG_WARNING = 4,
    LOG_ERROR = 3,
    LOG_CRITICAL = 2
  };

  CommManager(ROSflight& rf, CommLink& comm_link);

  void init();
  void receive(void);
  void stream();
  void send_param_value(uint16_t param_id);
  void set_streaming_rate(uint8_t stream_id, int16_t param_id);
  void update_status();
  void log(uint8_t severity, const char *fmt, ...);

  void send_named_value_float(const char *const name, float value);
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_COMM_MANAGER_H
