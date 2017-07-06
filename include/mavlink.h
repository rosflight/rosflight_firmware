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

#ifndef ROSFLIGHT_FIRMWARE_MAVLINK_H
#define ROSFLIGHT_FIRMWARE_MAVLINK_H

#include <mavlink/v1.0/rosflight/mavlink.h>
//#include "nanoprintf.h"

//#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))

//#define commlink_log(objectptr, severity, format, ...) do {\
//  char text[50]; \
//  nano_sprintf(text, format, ##__VA_ARGS__); \
//  objectptr->send_log_message(severity, text); \
//  } while(0)

//#define log_critical(objectptr, format, ...) commlink_log(objectptr, 2, format, ##__VA_ARGS__)
//#define log_error(objectptr, format, ...)    commlink_log(objectptr, 3, format, ##__VA_ARGS__)
//#define log_warning(objectptr, format, ...)  commlink_log(objectptr, 4, format, ##__VA_ARGS__)
//#define log_info(objectptr, format, ...)     commlink_log(objectptr, 6, format, ##__VA_ARGS__)

//#define mavlink_log_critical_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_critical(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

//#define mavlink_log_error_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_error(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

//#define mavlink_log_warning_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_warning(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

//#define mavlink_log_info_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_info(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

namespace rosflight_firmware {

class ROSflight;

class Mavlink
{
private:
  enum
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

  uint32_t sysid_;
  uint32_t compid_;
  uint64_t offboard_control_time_;
  ROSflight& RF_;
  uint8_t send_params_index_;
  mavlink_message_t in_buf_;
  mavlink_status_t status_;
  bool initialized_;

  typedef  void (Mavlink::*MavlinkStreamFcn)(void);

  typedef struct
  {
    uint32_t period_us;
    uint64_t next_time_us;
    MavlinkStreamFcn send_function;
  } mavlink_stream_t;

  void handle_msg_param_request_list(void);
  void handle_msg_param_request_read(const mavlink_message_t *const msg);
  void handle_msg_param_set(const mavlink_message_t *const msg);
  void send_next_param(void);

  void handle_mavlink_message(void);

  void handle_msg_rosflight_cmd(const mavlink_message_t *const msg);
  void handle_msg_timesync(const mavlink_message_t *const msg);
  void handle_msg_offboard_control(const mavlink_message_t *const msg);

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
  void send_message(const mavlink_message_t &msg);
  void stream_set_period(uint8_t stream_id, uint32_t period_us);

  // Debugging Utils
  void send_named_value_int(const char *const name, int32_t value);
  void send_named_value_float(const char *const name, float value);
//  void send_named_command_struct(const char *const name, control_t command_struct);

  mavlink_stream_t mavlink_streams_[STREAM_COUNT] = {
  //  period_us    last_time_us   send_function
    { 1000000,     0,             &rosflight_firmware::Mavlink::send_heartbeat },
    { 1000000,     0,             &rosflight_firmware::Mavlink::send_status},
    { 200000,      0,             &rosflight_firmware::Mavlink::send_attitude },
    { 1000,        0,             &rosflight_firmware::Mavlink::send_imu },
    { 200000,      0,             &rosflight_firmware::Mavlink::send_diff_pressure },
    { 200000,      0,             &rosflight_firmware::Mavlink::send_baro },
    { 100000,      0,             &rosflight_firmware::Mavlink::send_sonar },
    { 6250,        0,             &rosflight_firmware::Mavlink::send_mag },
    { 0,           0,             &rosflight_firmware::Mavlink::send_output_raw },
    { 0,           0,             &rosflight_firmware::Mavlink::send_rc_raw },
    { 5000,        0,             &rosflight_firmware::Mavlink::send_low_priority }
  };


public:
  Mavlink(ROSflight &_rf);

  void init();
  void receive(void);
  void stream();
  void update_param(uint16_t param_id);
  void set_streaming_rate(uint8_t stream_id, int16_t param_id);
  void send_log_message(uint8_t severity, char* text);
  void update_status();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_MAVLINK_H
