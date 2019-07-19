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

#include "test_board.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

namespace rosflight_firmware
{

void testBoard::set_rc(uint16_t *values)
{
  for (int i = 0; i < 8; i++)
  {
    rc_values[i] = values[i];
  }
}

void testBoard::set_time(uint64_t time_us)
{
  time_us_ = time_us;
}

void testBoard::set_pwm_lost(bool lost)
{
  rc_lost_ = lost;
}

void testBoard::set_imu(float *acc, float *gyro, uint64_t time_us)
{
  time_us_ = time_us;
  for (int i = 0; i < 3; i++)
  {
    acc_[i] = acc[i];
    gyro_[i] = gyro[i];
  }
  new_imu_ = true;
}


// setup
void testBoard::init_board() {}
void testBoard::board_reset(bool bootloader) {}

// clock
uint32_t testBoard::clock_millis() { return time_us_/1000; }
uint64_t testBoard::clock_micros() { return time_us_; }
void testBoard::clock_delay(uint32_t milliseconds) {}

// serial
void testBoard::serial_init(uint32_t baud_rate, uint32_t dev) {}
void testBoard::serial_write(const uint8_t *src, size_t len) {}
uint16_t testBoard::serial_bytes_available() { return 0; }
uint8_t testBoard::serial_read() { return 0; }
void testBoard::serial_flush() {}

// sensors
void testBoard::sensors_init() {}
uint16_t testBoard::num_sensor_errors() { return 0; }

bool testBoard::new_imu_data()
{
  if (new_imu_)
  {
    new_imu_ = false;
    return true;
  }
  return false;
}


bool testBoard::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time)
{
  for (int i = 0; i < 3; i++)
  {
    accel[i] = acc_[i];
    gyro[i] = gyro_[i];
  }
  *temperature = 25.0;
  *time = time_us_;
  return true;
}

void testBoard::imu_not_responding_error() {}

bool testBoard::mag_present() { return false; }
void testBoard::mag_update() {}
void testBoard::mag_read(float mag[3]) {}

bool testBoard::baro_present() { return false; }
void testBoard::baro_update() {}
void testBoard::baro_read(float *pressure, float *temperature) {}

bool testBoard::diff_pressure_present() { return false; }
void testBoard::diff_pressure_update() {}
void testBoard::diff_pressure_read(float *diff_pressure, float *temperature) {}

bool testBoard::sonar_present() { return false; }
void testBoard::sonar_update() {}
float testBoard::sonar_read() { return 0; }

//GNSS is not supported on the test board
GNSSData testBoard::gnss_read() { return {}; }

//GNSS is not supported on the test board
GNSSRaw testBoard::gnss_raw_read() { return {}; }

//GNSS is not supported on the test board
bool testBoard::gnss_has_new_data() { return false; }


// PWM
// TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
void testBoard::rc_init(rc_type_t rc_type) {}
bool testBoard::rc_lost() { return rc_lost_; }
float testBoard::rc_read(uint8_t channel)
{
  return static_cast<float>(rc_values[channel] - 1000)/1000.0 ;
}
void testBoard::pwm_write(uint8_t channel, float value) {}
void testBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) {}
void testBoard::pwm_disable() {}

// non-volatile memory
void testBoard::memory_init() {}
bool testBoard::memory_read(void *dest, size_t len) { return false; }
bool testBoard::memory_write(const void *src, size_t len) { return false; }

// LEDs
void testBoard::led0_on() {}
void testBoard::led0_off() {}
void testBoard::led0_toggle() {}

void testBoard::led1_on() {}
void testBoard::led1_off() {}
void testBoard::led1_toggle() {}

//Backup memory
bool testBoard::has_backup_data() { return false; }
BackupData testBoard::get_backup_data()
{
#pragma GCC diagnostic push //Ignore blank fields in struct
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  BackupData blank_data = {0};
#pragma GCC diagnostic pop
  return blank_data;
}

} // namespace rosflight_firmware

#pragma GCC diagnostic pop
