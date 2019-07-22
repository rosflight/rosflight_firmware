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

#ifndef ROSFLIGHT_FIRMWARE_COMM_LINK_H
#define ROSFLIGHT_FIRMWARE_COMM_LINK_H

#include <cstdint>

#include <turbomath/turbomath.h>

#include "param.h"
#include "board.h"
#include "sensors.h"

namespace rosflight_firmware
{

class CommLinkInterface
{
public:

  enum class LogSeverity
  {
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_CRITICAL
  };

  enum class Command
  {
    COMMAND_READ_PARAMS,
    COMMAND_WRITE_PARAMS,
    COMMAND_SET_PARAM_DEFAULTS,
    COMMAND_ACCEL_CALIBRATION,
    COMMAND_GYRO_CALIBRATION,
    COMMAND_BARO_CALIBRATION,
    COMMAND_AIRSPEED_CALIBRATION,
    COMMAND_RC_CALIBRATION,
    COMMAND_REBOOT,
    COMMAND_REBOOT_TO_BOOTLOADER,
    COMMAND_SEND_VERSION
  };

  struct OffboardControl
  {
    enum class Mode
    {
      PASS_THROUGH,
      ROLLRATE_PITCHRATE_YAWRATE_THROTTLE,
      ROLL_PITCH_YAWRATE_THROTTLE
    };

    struct Channel
    {
      float value;
      bool valid;
    };

    Mode mode;
    Channel x;
    Channel y;
    Channel z;
    Channel F;
  };

  struct AuxCommand
  {
    enum class Type
    {
      DISABLED,
      SERVO,
      MOTOR
    };

    struct AuxChannel
    {
      Type type;
      float value;
    };

    AuxChannel cmd_array[14];
  };

    class ListenerInterface
    {
    public:
      virtual void param_request_list_callback(uint8_t target_system) = 0;
      virtual void param_request_read_callback(uint8_t target_system, const char *const param_name, int16_t param_index) = 0;
      virtual void param_set_int_callback(uint8_t target_system, const char *const param_name, int32_t param_value) = 0;
      virtual void param_set_float_callback(uint8_t target_system, const char *const param_name, float param_value) = 0;
      virtual void command_callback(Command command) = 0;
      virtual void timesync_callback(int64_t tc1, int64_t ts1) = 0;
      virtual void offboard_control_callback(const OffboardControl &control) = 0;
      virtual void aux_command_callback(const AuxCommand &command) = 0;
      virtual void attitude_correction_callback(const turbomath::Quaternion &q) = 0;
      virtual void heartbeat_callback() = 0;
    };

    virtual void init(uint32_t baud_rate, uint32_t dev) = 0;
    virtual void receive() = 0;

    // send functions

    virtual void send_attitude_quaternion(uint8_t system_id,
                                          uint64_t timestamp_us,
                                          const turbomath::Quaternion &attitude,
                                          const turbomath::Vector &angular_velocity) = 0;
    virtual void send_baro(uint8_t system_id, float altitude, float pressure, float temperature) = 0;
    virtual void send_command_ack(uint8_t system_id, Command command, bool success) = 0;
    virtual void send_diff_pressure(uint8_t system_id, float velocity, float pressure, float temperature) = 0;
    virtual void send_heartbeat(uint8_t system_id, bool fixed_wing) = 0;
    virtual void send_imu(uint8_t system_id,
                          uint64_t timestamp_us,
                          const turbomath::Vector &accel,
                          const turbomath::Vector &gyro,
                          float temperature) = 0;
    virtual void send_log_message(uint8_t system_id, LogSeverity severity, const char *text) = 0;
    virtual void send_mag(uint8_t system_id, const turbomath::Vector &mag) = 0;
    virtual void send_named_value_int(uint8_t system_id, uint32_t timestamp_ms, const char *const name, int32_t value) = 0;
    virtual void send_named_value_float(uint8_t system_id, uint32_t timestamp_ms, const char *const name, float value) = 0;
    virtual void send_output_raw(uint8_t system_id, uint32_t timestamp_ms, const float raw_outputs[14]) = 0;
    virtual void send_param_value_int(uint8_t system_id,
                                      uint16_t index,
                                      const char *const name,
                                      int32_t value,
                                      uint16_t param_count) = 0;
    virtual void send_param_value_float(uint8_t system_id,
                                        uint16_t index,
                                        const char *const name,
                                        float value,
                                        uint16_t param_count) = 0;
    virtual void send_rc_raw(uint8_t system_id, uint32_t timestamp_ms, const uint16_t channels[8]) = 0;
    virtual void send_sonar(uint8_t system_id, /* TODO enum type*/ uint8_t type, float range, float max_range, float min_range) = 0;
    virtual void send_status(uint8_t system_id,
                             bool armed,
                             bool failsafe,
                             bool rc_override,
                             bool offboard,
                             uint8_t error_code,
                             uint8_t control_mode,
                             int16_t num_errors,
                             int16_t loop_time_us) = 0;
    virtual void send_timesync(uint8_t system_id, int64_t tc1, int64_t ts1) = 0;
    virtual void send_version(uint8_t system_id, const char *const version) = 0;
    virtual void send_gnss(uint8_t system_id, const GNSSData &data) = 0;
    virtual void send_gnss_raw(uint8_t system_id, const GNSSRaw &data) = 0;
    virtual void send_error_data(uint8_t system_id, const BackupData &error_data) = 0;

    // register listener
    virtual void set_listener(ListenerInterface *listener) = 0;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_COMM_LINK_H
