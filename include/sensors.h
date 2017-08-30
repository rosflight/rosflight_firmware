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

#include <stdint.h>
#include <stdbool.h>
#include <turbomath/turbomath.h>

namespace rosflight_firmware
{

class ROSflight;

class Sensors
{
public:
  struct Data
  {
    turbomath::Vector accel = {0, 0, 0};
    turbomath::Vector gyro = {0, 0, 0};
    turbomath::Quaternion fcu_orientation = {1, 0, 0, 0};
    float imu_temperature = 0;
    uint64_t imu_time = 0;

    float diff_pressure_velocity = 0;
    float diff_pressure = 0;
    float diff_pressure_temp = 0;
    bool diff_pressure_valid = false;

    float baro_altitude = 0;
    float baro_pressure = 0;
    float baro_temperature = 0;
    bool baro_valid = false;

    float sonar_range = 0;
    bool sonar_range_valid = false;

    turbomath::Vector mag = {0, 0, 0};

    bool baro_present = false;
    bool mag_present = false;
    bool sonar_present = false;
    bool diff_pressure_present = false;
  };

  Sensors(ROSflight& rosflight);

  inline const Data& data() const { return data_; }

  // function declarations
  void init();
  bool run();

  // Calibration Functions
  bool start_imu_calibration(void);
  bool start_gyro_calibration(void);
  bool start_baro_calibration(void);
  bool start_diff_pressure_calibration(void);
  bool gyro_calibration_complete(void);

  inline bool should_send_imu_data(void)
  {
    if (imu_data_sent_)
      return false;
    else
      imu_data_sent_ = true;
    return true;
  }

private:
  static const float BARO_MAX_CHANGE_RATE;
  static const float BARO_SAMPLE_RATE;
  static const float DIFF_MAX_CHANGE_RATE;
  static const float DIFF_SAMPLE_RATE;
  static const float SONAR_MAX_CHANGE_RATE;
  static const float SONAR_SAMPLE_RATE;
  static const int SENSOR_CAL_DELAY_CYCLES;
  static const int SENSOR_CAL_CYCLES;
  static const float BARO_MAX_CALIBRATION_VARIANCE;
  static const float DIFF_PRESSURE_MAX_CALIBRATION_VARIANCE;

  class OutlierFilter
  {
  private:
    float max_change_;
    float center_;
    int window_size_;
    bool init_ = false;

  public:
    OutlierFilter() {};
    void init(float max_change_rate, float update_rate, float center);
    bool update(float new_val, float *val);
  };

  enum LowPrioritySensors
  {
    BAROMETER,
    DIFF_PRESSURE,
    SONAR,
    MAGNETOMETER,
    NUM_LOW_PRIORITY_SENSORS
  };

  ROSflight& rf_;

  Data data_;

  float accel_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};

  bool calibrating_acc_flag_ = false;
  bool calibrating_gyro_flag_ = false;
  LowPrioritySensors next_sensor_to_update_ = BAROMETER;
  void calibrate_accel(void);
  void calibrate_gyro(void);
  void calibrate_baro(void);
  void calibrate_diff_pressure(void);
  void correct_imu(void);
  void correct_mag(void);
  void correct_baro(void);
  void correct_diff_pressure(void);
  bool update_imu(void);
  void update_other_sensors(void);
  void look_for_disabled_sensors(void);
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

  // Baro Calibration
  bool baro_calibrated_ = false;
  float ground_pressure_ = 0.0f;
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

  // Sensor Measurement Outlier Filters
  OutlierFilter baro_outlier_filt_;
  OutlierFilter diff_outlier_filt_;
  OutlierFilter sonar_outlier_filt_;

};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_SENSORS_H
