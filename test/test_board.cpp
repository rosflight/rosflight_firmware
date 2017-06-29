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

namespace rosflight_firmware
{


// setup
  void testBoard::init_board(void){}
  void testBoard::board_reset(bool bootloader){}

// clock
  uint32_t testBoard::clock_millis(){}
  uint64_t testBoard::clock_micros(){}
  void testBoard::clock_delay(uint32_t milliseconds){}

// serial
  void testBoard::serial_init(uint32_t baud_rate){}
  void testBoard::serial_write(uint8_t byte){}
  uint16_t testBoard::serial_bytes_available(void){}
  uint8_t testBoard::serial_read(void){}

// sensors
  void testBoard::sensors_init(){}
  uint16_t testBoard::num_sensor_errors(void) {}

  bool testBoard::new_imu_data(){}
  void testBoard::imu_read_accel(float accel[3]){}
  void testBoard::imu_read_gyro(float gyro[3]){}
  bool testBoard::imu_read_all(float accel[3], float *temperature, float gyro[3], uint64_t* time){}
  float testBoard::imu_read_temperature(void){}
  void testBoard::imu_not_responding_error(void){}

  bool testBoard::mag_check(void){}
  bool testBoard::mag_present(void){}
  void testBoard::mag_read(float mag[3]){}

  bool testBoard::baro_present(void){}
  bool testBoard::baro_check(void){}
  void testBoard::baro_read(float *altitude, float *pressure, float *temperature) {}
  void testBoard::baro_calibrate(){}

  bool testBoard::diff_pressure_present(void){}
  bool testBoard::diff_pressure_check(void){}
  void testBoard::diff_pressure_set_atm(float barometric_pressure){}
  void testBoard::diff_pressure_calibrate(){}
  void testBoard::diff_pressure_read(float *diff_pressure, float *temperature, float *velocity) {}

  bool testBoard::sonar_present(void){}
  bool testBoard::sonar_check(void){}
  float testBoard::sonar_read(void){}

// PWM
// TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void testBoard::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm){}
  bool testBoard::pwm_lost(){}
  uint16_t testBoard::pwm_read(uint8_t channel){}
  void testBoard::pwm_write(uint8_t channel, uint16_t value){}

// non-volatile memory
  void testBoard::memory_init(void){}
  bool testBoard::memory_read(void *dest, size_t len){}
  bool testBoard::memory_write(const void *src, size_t len){}

// LEDs
  void testBoard::led0_on(void){}
  void testBoard::led0_off(void){}
  void testBoard::led0_toggle(void){}

  void testBoard::led1_on(void){}
  void testBoard::led1_off(void){}
  void testBoard::led1_toggle(void){}

} // namespace rosflight_firmware
