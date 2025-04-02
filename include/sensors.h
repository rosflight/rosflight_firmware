/*
 * Copyright (c) 2017, James Jackson, Daniel Koch, and Craig Bidstrup,
 * BYU MAGICC Lab
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

#ifndef ROSFLIGHT_FIRMWARE_SENSORS_H
#define ROSFLIGHT_FIRMWARE_SENSORS_H

#include "interface/param_listener.h"

#include <turbomath/turbomath.h>

#include <cstdbool>
#include <cstdint>
#include <cstring>

namespace rosflight_firmware
{
typedef struct
{
  bool imu;
  bool gnss;
  bool baro;
  bool mag;
  bool diff_pressure;
  bool sonar;
  bool battery;
  bool rc;
} got_flags;

enum GNSSFixType
{
  GNSS_FIX_TYPE_NO_FIX = 0,
  GNSS_FIX_TYPE_DEAD_RECKONING_ONLY = 1,
  GNSS_FIX_TYPE_2D_FIX = 2,
  GNSS_FIX_TYPE_3D_FIX = 3,
  GNSS_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING = 4,
  GNSS_FIX_TYPE_TIME_FIX_ONLY = 5,
};

struct GNSSData
{
  uint64_t time_of_week;  // Used internally to firmware for checking gnss
                          // messages. Not included in ROS2 or MAVlink messages
  uint64_t seconds;  // Unix time, in seconds
  uint64_t nanos; // Fractional time
  GNSSFixType fix_type;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t num_sat;
  double lon;
  double lat;
  float height;
  float vel_n;  // mm/s
  float vel_e;  // mm/s
  float vel_d;  // mm/s
  float h_acc; // mm
  float v_acc; // mm
  float s_acc; // mm/s

  GNSSData() { memset(this, 0, sizeof(GNSSData)); }
};

class ROSflight;

class Sensors : public ParamListenerInterface
{
public:
  struct Data
  {
    turbomath::Vector accel = {0, 0, 0};
    turbomath::Vector gyro = {0, 0, 0};
    turbomath::Quaternion fcu_orientation = {1, 0, 0, 0};
    float imu_temperature = 0;
    uint64_t imu_time = 0;

    float diff_pressure_ias = 0;
    float diff_pressure = 0;
    float diff_pressure_temp = 0;

    float baro_altitude = 0;
    float baro_pressure = 0;
    float baro_temperature = 0;

    float sonar_range = 0;
    bool sonar_range_valid = false;

    GNSSData gnss_data;
    float gps_CNO = 0; // What is this?
    bool gnss_present = false;

    turbomath::Vector mag = {0, 0, 0};

    bool baro_present = false;
    bool mag_present = false;
    bool sonar_present = false;
    bool diff_pressure_present = false;

    bool battery_monitor_present = false;
    float battery_voltage = 0;
    float battery_current = 0;
  };

  Sensors(ROSflight & rosflight);

  inline const Data & data() const { return data_; }
  inline float rho() { return rho_; }
  void get_filtered_IMU(turbomath::Vector & accel, turbomath::Vector & gyro, uint64_t & stamp_us);

  // function declarations
  void init();
  got_flags run();
  void param_change_callback(uint16_t param_id) override;

  // Calibration Functions
  bool start_imu_calibration(void);
  bool start_gyro_calibration(void);
  bool start_baro_calibration(void);
  bool start_diff_pressure_calibration(void);
  bool gyro_calibration_complete(void);

  inline bool should_send_imu_data(void)
  {
    if (imu_data_sent_) {
      return false;
    } else {
      imu_data_sent_ = true;
    }
    return true;
  }

  got_flags got;

private:
  static const int SENSOR_CAL_DELAY_CYCLES;
  static const int SENSOR_CAL_CYCLES;
  static const float BARO_MAX_CALIBRATION_VARIANCE;
  static const float DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE;

  enum : uint8_t
  {
    BAROMETER,
    GNSS,
    DIFF_PRESSURE,
    SONAR,
    MAGNETOMETER,
    BATTERY_MONITOR,
    NUM_LOW_PRIORITY_SENSORS
  };

  ROSflight & rf_;

  Data data_;

  float accel_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};

  float rho_ = 1.225;

  bool calibrating_acc_flag_ = false;
  bool calibrating_gyro_flag_ = false;
  uint8_t next_sensor_to_update_ = BAROMETER;
  void init_imu();
  void calibrate_accel(void);
  void calibrate_gyro(void);
  void calibrate_baro(void);
  void calibrate_diff_pressure(void);
  void correct_imu(void);
  void correct_mag(void);
  void correct_baro(void);
  void correct_diff_pressure(void);
  bool update_imu(void);
  void update_battery_monitor(void);
  void update_other_sensors(void);
  void look_for_disabled_sensors(void);
  void update_battery_monitor_multipliers(void);
  uint32_t last_time_look_for_disarmed_sensors_ = 0;
  uint32_t last_imu_update_ms_ = 0;

  bool new_imu_data_;
  bool imu_data_sent_;

  // IMU calibration
  uint16_t gyro_calibration_count_ = 0;
  turbomath::Vector gyro_sum_ = {0, 0, 0};
  uint16_t accel_calibration_count_ = 0;
  turbomath::Vector acc_sum_ = {0, 0, 0};
  const turbomath::Vector gravity_ = {0.0f, 0.0f, 9.80665f};
  float acc_temp_sum_ = 0.0f;
  turbomath::Vector max_ = {-1000.0f, -1000.0f, -1000.0f};
  turbomath::Vector min_ = {1000.0f, 1000.0f, 1000.0f};

  // Filtered IMU
  turbomath::Vector accel_int_;
  turbomath::Vector gyro_int_;
  uint64_t int_start_us_;
  uint64_t prev_imu_read_time_us_;

  // Baro Calibration
  bool baro_calibrated_ = false;
  uint16_t baro_calibration_count_ = 0;
  uint32_t last_baro_cal_iter_ms_ = 0;
  float baro_calibration_mean_ = 0.0f;
  float baro_calibration_var_ = 0.0f;

  // Diff Pressure Calibration
  bool diff_pressure_calibrated_ = false;
  uint16_t diff_pressure_calibration_count_ = 0;
  uint32_t last_diff_pressure_cal_iter_ms_ = 0;
  float diff_pressure_calibration_mean_ = 0.0f;
  float diff_pressure_calibration_var_ = 0.0f;

  uint32_t last_battery_monitor_update_ms_ = 0;
  // Battery Monitor
  float battery_voltage_alpha_{0.995};
  float battery_current_alpha_{0.995};
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_SENSORS_H
