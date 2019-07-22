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

#ifndef ROSFLIGHT_FIRMWARE_TEST_BOARD_H
#define ROSFLIGHT_FIRMWARE_TEST_BOARD_H

#include "board.h"
#include "sensors.h"

namespace rosflight_firmware
{

class testBoard : public Board
{

private:
  uint16_t rc_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint64_t time_us_ = 0;
  bool rc_lost_ = false;
  float acc_[3] = {0, 0, 0};
  float gyro_[3] = {0, 0, 0};
  bool new_imu_ = false;

public:
// setup
  void init_board() override;
  void board_reset(bool bootloader) override;

// clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

// serial
  void serial_init(uint32_t baud_rate, uint32_t dev) override;
  void serial_write(const uint8_t *src, size_t len) override;
  uint16_t serial_bytes_available() override;
  uint8_t serial_read() override;
  void serial_flush() override;

// sensors
  void sensors_init() override;
  uint16_t num_sensor_errors() ;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time) override;
  void imu_not_responding_error() override;

  bool mag_present() override;
  void mag_update() override;
  void mag_read(float mag[3]) override;

  bool baro_present() override;
  void baro_update() override;
  void baro_read(float *pressure, float *temperature) override;

  bool diff_pressure_present() override;
  void diff_pressure_update() override;
  void diff_pressure_read(float *diff_pressure, float *temperature) override;

  bool sonar_present() override;
  void sonar_update() override;
  float sonar_read() override;

  bool gnss_present() override { return false; }
  void gnss_update() override {}
  GNSSData gnss_read() override;
  GNSSRaw gnss_raw_read() override;
  bool gnss_has_new_data() override;


// RC
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost() override;
  float rc_read(uint8_t channel) override;

// PWM
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  void pwm_disable() override;
  void pwm_write(uint8_t channel, float value) override;

// non-volatile memory
  void memory_init() override;
  bool memory_read(void *dest, size_t len) override;
  bool memory_write(const void *src, size_t len) override;

// LEDs
  void led0_on() override;
  void led0_off() override;
  void led0_toggle() override;

  void led1_on() override;
  void led1_off() override;
  void led1_toggle() override;

//Backup memory
  bool has_backup_data() override;
  BackupData get_backup_data() override;


  void set_imu(float *acc, float *gyro, uint64_t time_us);
  void set_rc(uint16_t *values);
  void set_time(uint64_t time_us);
  void set_pwm_lost(bool lost);

};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_TEST_BOARD_H
