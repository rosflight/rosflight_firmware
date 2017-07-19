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

#include <functional>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

namespace rosflight_firmware
{

class Board
{

public:
// setup
  virtual void init_board(void) = 0;
  virtual void board_reset(bool bootloader) = 0;

// clock
  virtual uint32_t clock_millis() = 0;
  virtual uint64_t clock_micros() = 0;
  virtual void clock_delay(uint32_t milliseconds) = 0;

// serial
  virtual void serial_init(uint32_t baud_rate) = 0;
  virtual void serial_write(const uint8_t *src, size_t len) = 0;
  virtual uint16_t serial_bytes_available(void) = 0;
  virtual uint8_t serial_read(void) = 0;

// sensors
  virtual void sensors_init() = 0;
  virtual uint16_t num_sensor_errors(void)  = 0;

  virtual bool new_imu_data() = 0;
  virtual bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t* time) = 0;
  virtual void imu_not_responding_error(void) = 0;

  virtual bool mag_check(void) = 0;
  virtual void mag_read(float mag[3]) = 0;

  virtual bool baro_check(void) = 0;
  virtual void baro_read(float *pressure, float *temperature) = 0;

  virtual bool diff_pressure_check(void) = 0;
  virtual void diff_pressure_read(float *diff_pressure, float *temperature) = 0;

  virtual bool sonar_check(void) = 0;
  virtual float sonar_read(void) = 0;

// PWM
// TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
  virtual void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm) = 0;
  virtual bool pwm_lost() = 0;
  virtual uint16_t pwm_read(uint8_t channel) = 0;
  virtual void pwm_write(uint8_t channel, uint16_t value) = 0;

// non-volatile memory
  virtual void memory_init(void) = 0;
  virtual bool memory_read(void *dest, size_t len) = 0;
  virtual bool memory_write(const void *src, size_t len) = 0;

// LEDs
  virtual void led0_on(void) = 0;
  virtual void led0_off(void) = 0;
  virtual void led0_toggle(void) = 0;

  virtual void led1_on(void) = 0;
  virtual void led1_off(void) = 0;
  virtual void led1_toggle(void) = 0;

};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_BOARD_H
