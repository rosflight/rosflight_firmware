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

#ifndef ROSFLIGHT_FIRMWARE_BOARD_H
#define ROSFLIGHT_FIRMWARE_BOARD_H

#include <cstdbool>
#include <cstddef>
#include <cstdint>

#include "board.h"

namespace rosflight_firmware
{

typedef struct //__attribute__((__packed__))
{
  uint64_t timestamp; // us, time of data read complete
  uint64_t complete;  //
  uint16_t status;    // device dependent
} PacketHeader;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float voltage;
  float current;
  float temperature; // STM32 temperature, not batter temperature
} BatteryStruct;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float accel[3];    // rad/s
  float gyro[3];     // rad/s
  float temperature; // K
} ImuStruct;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float pressure;    // Pa
  float temperature; // K
  union
  {
    float altitude;
    float ias; //speed;
  };
} PressureStruct;

enum class SensorRangeType // c.f., ROSFLIGHT_RANGE_TYPE
{
  ROSFLIGHT_RANGE_SONAR = 0, /*  | */
  ROSFLIGHT_RANGE_LIDAR = 1, /*  | */
  END = 2,                   /*  | */
};

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float range;          // m
  float min_range;      // m
  float max_range;      // m
  SensorRangeType type; // ROSFLIGHT_RANGE_SONAR, ROSFLIGHT_RANGE_SONAR
} RangeStruct;

typedef struct //__attribute__((packed))
{
  PacketHeader header; //
  float flux[3];       // T, magnetic flux density
  float temperature;   // K
} MagStruct;

enum class GNSSFixType // quality from GGA
{
  GNSS_FIX_TYPE_NO_FIX = 0,
  GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1,
  GNSS_FIX_TYPE_2D_FIX = 2,
  GNSS_FIX_TYPE_3D_FIX = 3,
  GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4,
  GNSS_FIX_TYPE_TIME_FIX_ONLY = 5,
  GNSS_FIX_RTK_FLOAT = 6,
  GNSS_FIX_RTK_FIXED = 7,
  END = 8
};

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  uint64_t pps;           // most recent pps timestamp (us)
  int64_t unix_seconds;   // Unix time, in seconds
  int32_t unix_nanos;
  uint8_t fix_type;
  uint8_t num_sat;
  double lon;
  double lat;
  float height_msl;
  float vel_n;
  float vel_e;
  float vel_d;
  float h_acc;
  float v_acc;
  float speed_accy;
} GnssStruct;

typedef struct
{
  bool imu;
  bool gnss;
  bool baro;
  bool mag;
  bool diff_pressure;
  bool sonar;
  bool battery;
} got_flags;

// 16 analog + 8 digital MUST BE > 14 (Mavlink message size is hardware to 14)
#define RC_STRUCT_CHANNELS 24
typedef struct //__attribute__((packed))
{
  PacketHeader header;
  uint8_t nChan;
  float chan[RC_STRUCT_CHANNELS];
  bool frameLost;
  bool failsafeActivated;
} RcStruct;

typedef struct //__attribute__((__packed__))
{
  PacketHeader header;
  float q[4]; // quaternions
  float rate[3];
} AttitudeStruct;

class Board
{
public:
  typedef enum
  {
    RC_TYPE_PPM = 0,
    RC_TYPE_SBUS = 1
  } rc_type_t;

  // setup
  virtual void init_board() = 0;
  virtual void board_reset(bool bootloader) = 0;

  virtual void sensors_init(void) = 0;
  virtual uint16_t sensors_errors_count() = 0;

  virtual uint16_t sensors_init_message_count() =  0;
  virtual bool sensors_init_message_good(uint16_t i) = 0;
  virtual uint16_t sensors_init_message(char *message, uint16_t size, uint16_t i) = 0;

  // clock
  virtual uint32_t clock_millis() = 0;
  virtual uint64_t clock_micros() = 0;
  virtual void clock_delay(uint32_t milliseconds) = 0;

  // serial
  virtual void serial_init(uint32_t baud_rate, uint32_t dev) = 0;
  // qos defines the 'priority' of the packet, with 0 being the highest
  virtual void serial_write(const uint8_t * src, size_t len, uint8_t qos) = 0;
  virtual uint16_t serial_bytes_available() = 0;
  virtual uint8_t serial_read() = 0;
  virtual void serial_flush() = 0;

  // IMU
  virtual bool imu_read(rosflight_firmware::ImuStruct * imu) = 0;

  // Mag
  virtual bool mag_read(rosflight_firmware::MagStruct * mag) = 0;

  // Baro
  virtual bool baro_read(PressureStruct * diff_pressure) = 0;

  // Pitot
  virtual bool diff_pressure_read(PressureStruct * diff_pressure) = 0;

  // Sonar
  virtual bool sonar_read(RangeStruct * sonar) = 0;

  // GPS
  virtual bool gnss_read(rosflight_firmware::GnssStruct * gnss) = 0;
  // Battery
  virtual bool battery_read(rosflight_firmware::BatteryStruct * bat) = 0;
  virtual void battery_voltage_set_multiplier(double multiplier) = 0;
  virtual void battery_current_set_multiplier(double multiplier) = 0;

  // RC
  virtual void rc_init(rc_type_t rc_type) = 0;
  virtual bool rc_read(rosflight_firmware::RcStruct * rc) = 0;

  // PWM
  virtual void pwm_init(const float * rate, uint32_t channels) = 0;
  virtual void pwm_disable() = 0;
  virtual void pwm_write(float * value, uint32_t channels) = 0;

  // non-volatile memory
  virtual void memory_init() = 0;
  virtual bool memory_read(void * dest, size_t len) = 0;
  virtual bool memory_write(const void * src, size_t len) = 0;

  // LEDs
  virtual void led0_on() = 0;
  virtual void led0_off() = 0;
  virtual void led0_toggle() = 0;

  virtual void led1_on() = 0;
  virtual void led1_off() = 0;
  virtual void led1_toggle() = 0;

  // Backup memory
  virtual void backup_memory_init() = 0;
  virtual bool backup_memory_read(void * dest, size_t len) = 0;
  virtual void backup_memory_write(const void * src, size_t len) = 0;
  virtual void backup_memory_clear(size_t len) = 0;
};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_BOARD_H
