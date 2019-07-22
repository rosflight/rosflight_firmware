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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensors.h"
#include "state_manager.h"

namespace rosflight_firmware
{

struct debug_info_t
{
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
};

struct BackupData
{
  uint32_t error_code;
  debug_info_t debug_info;
  uint32_t reset_count;
  uint32_t arm_status; //This must equals ARM_MAGIC, or else the state manager will not rearm on reboot
  //TODO add state manager info
  StateManager::State state;
  uint32_t checksum; //With the current implementation of the checksum, this must go last
};

//This magic number is used to check that the firmware was armed before it reset
const uint32_t ARM_MAGIC = 0xfa11bad;

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

// clock
  virtual uint32_t clock_millis() = 0;
  virtual uint64_t clock_micros() = 0;
  virtual void clock_delay(uint32_t milliseconds) = 0;

// serial
  virtual void serial_init(uint32_t baud_rate, uint32_t dev) = 0;
  virtual void serial_write(const uint8_t *src, size_t len) = 0;
  virtual uint16_t serial_bytes_available() = 0;
  virtual uint8_t serial_read() = 0;
  virtual void serial_flush() = 0;

// sensors
  virtual void sensors_init() = 0;
  virtual uint16_t num_sensor_errors()  = 0;

  virtual bool new_imu_data() = 0;
  virtual bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time) = 0;
  virtual void imu_not_responding_error() = 0;

  virtual bool mag_present() = 0;
  virtual void mag_update() = 0;
  virtual void mag_read(float mag[3]) = 0;

  virtual bool baro_present() = 0;
  virtual void baro_update() = 0;
  virtual void baro_read(float *pressure, float *temperature) = 0;

  virtual bool diff_pressure_present() = 0;
  virtual void diff_pressure_update() = 0;
  virtual void diff_pressure_read(float *diff_pressure, float *temperature) = 0;

  virtual bool sonar_present() = 0;
  virtual void sonar_update() = 0;
  virtual float sonar_read() = 0;

  virtual bool gnss_present() = 0;
  virtual void gnss_update() = 0;

  virtual GNSSData gnss_read() = 0;
  virtual bool gnss_has_new_data() = 0;
  virtual GNSSRaw gnss_raw_read() = 0;

// RC
  virtual void rc_init(rc_type_t rc_type) = 0;
  virtual bool rc_lost() = 0;
  virtual float rc_read(uint8_t channel) = 0;

// PWM
  virtual void pwm_init(uint32_t refresh_rate, uint16_t  idle_pwm) = 0;
  virtual void pwm_disable() = 0;
  virtual void pwm_write(uint8_t channel, float value) = 0;

// non-volatile memory
  virtual void memory_init() = 0;
  virtual bool memory_read(void *dest, size_t len) = 0;
  virtual bool memory_write(const void *src, size_t len) = 0;

// LEDs
  virtual void led0_on() = 0;
  virtual void led0_off() = 0;
  virtual void led0_toggle() = 0;

  virtual void led1_on() = 0;
  virtual void led1_off() = 0;
  virtual void led1_toggle() = 0;

// Backup memory
  virtual bool has_backup_data() = 0;
  virtual BackupData get_backup_data() = 0;

};

} // namespace rosflight_firmware

#endif // ROSLFIGHT_FIRMWARE_BOARD_H
