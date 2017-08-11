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
  void init_board(void);
  void board_reset(bool bootloader);

// clock
  uint32_t clock_millis();
  uint64_t clock_micros();
  void clock_delay(uint32_t milliseconds);

// serial
  void serial_init(uint32_t baud_rate);
  void serial_write(const uint8_t *src, size_t len);
  uint16_t serial_bytes_available(void);
  uint8_t serial_read(void);

// sensors
  void sensors_init();
  uint16_t num_sensor_errors(void) ;

  bool new_imu_data();
  bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t* time);
  void imu_not_responding_error(void);

  bool mag_check(void);
  void mag_read(float mag[3]);

  bool baro_check(void);
  void baro_read(float *pressure, float *temperature);

  bool diff_pressure_check(void);
  void diff_pressure_read(float *diff_pressure, float *temperature);

  bool sonar_check(void);
  float sonar_read(void);

// PWM
// TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm);
  bool pwm_lost();
  uint16_t pwm_read(uint8_t channel);
  void pwm_write(uint8_t channel, uint16_t value);

// non-volatile memory
  void memory_init(void);
  bool memory_read(void *dest, size_t len);
  bool memory_write(const void *src, size_t len);

// LEDs
  void led0_on(void);
  void led0_off(void);
  void led0_toggle(void);

  void led1_on(void);
  void led1_off(void);
  void led1_toggle(void);



  void set_imu(float* acc, float* gyro, uint64_t time_us);
  void set_rc(uint16_t* values);
  void set_time(uint64_t time_us);
  void set_pwm_lost(bool lost);

};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_TEST_BOARD_H
