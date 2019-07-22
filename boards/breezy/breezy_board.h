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

#ifndef ROSFLIGHT_FIRMWARE_BREEZY_BOARD_H
#define ROSFLIGHT_FIRMWARE_BREEZY_BOARD_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

extern "C"
{
#include <breezystm32.h>
}

#include "board.h"
#include "sensors.h"

namespace rosflight_firmware
{

class BreezyBoard : public Board
{

private:
  serialPort_t *Serial1;

  int _board_revision = 2;

  float _accel_scale = 1.0;
  float _gyro_scale = 1.0;

  enum
  {
    SONAR_NONE,
    SONAR_I2C,
    SONAR_PWM
  };
  uint8_t sonar_type = SONAR_NONE;

  rc_type_t rc_type_ = RC_TYPE_PPM;
  uint32_t pwm_refresh_rate_ = 490;
  uint16_t pwm_idle_pwm_ = 1000;
  enum
  {
    BARO_NONE,
    BARO_BMP280,
    BARO_MS5611
  };
  uint8_t baro_type = BARO_NONE;

  bool new_imu_data_;
  uint64_t imu_time_us_;

public:
  BreezyBoard();

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
  uint16_t num_sensor_errors() override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us) override;
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

  bool gnss_present() override
  {
    return false;
  }

  void gnss_update() override
  {
    return;
  }

  GNSSData gnss_read() override;
  bool gnss_has_new_data() override;
  GNSSRaw gnss_raw_read() override;


  // PWM
  // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost() override;
  float rc_read(uint8_t channel) override;

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

  // Backup memory
  bool has_backup_data() override;
  BackupData get_backup_data() override;
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_BREEZY_BOARD_H
