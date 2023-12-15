/**
 ******************************************************************************
 * File     : varmint.h
 * Date     : Sep 27, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023, AeroVironment, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1.Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2.Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3.Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 *
 ******************************************************************************
 **/

#ifndef VARMINT_H_
#define VARMINT_H_

#include <Adc.h>
#include <Adis165xx.h>
#include <Bmi088.h>
#include <DlhrL20G.h>
#include <Dps310.h>
#include <Iis2mdc.h>
#include <Pwm.h>
#include <Sbus.h>
#include <Sd.h>
#include <Telem.h>
#include <Ubx.h>
#include <Vcp.h>
#include <board.h>

#ifdef __cplusplus
extern "C"
{
#endif

  int varmint_main(void);

#ifdef __cplusplus
}
#endif

/*
 *
 */
class Varmint : public rosflight_firmware::Board
{
  /**
   * \brief
   *
   *
   */
private:
  uint32_t serial_device_;

public:
  Varmint(){};

  Adis165xx imu0_;
  Bmi088 imu1_;
  DlhrL20G pitot_;
  Dps310 baro_;
  Iis2mdc mag_;
  Sbus rc_;
  Ubx gps_;
  Adc adc_;
  Telem telem_;
  Vcp vcp_;
  Pwm pwm_[PWM_CHANNELS];
  Sd sd_;

  // Required ROSflight Board HAL functions:

  // setup

  void init_board(void) override;
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

  bool imu_has_new_data() override;
  bool imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us) override;
  void imu_not_responding_error() override;

  bool mag_present() override;
  bool mag_has_new_data() override;
  bool mag_read(float mag[3]) override;

  bool baro_present() override;
  bool baro_has_new_data() override;
  bool baro_read(float *pressure, float *temperature) override;

  bool diff_pressure_present() override;
  bool diff_pressure_has_new_data() override;
  bool diff_pressure_read(float *diff_pressure, float *temperature) override;

  bool sonar_present() override;
  bool sonar_has_new_data() override;
  bool sonar_read(float *range) override;

  // Battery
  bool battery_has_new_data() override;
  bool battery_read(float *voltage, float *current) override;
  bool battery_present() override;
  void battery_voltage_set_multiplier(double multiplier) override;
  void battery_current_set_multiplier(double multiplier) override;

  // GNSS
  bool gnss_present() override;
  bool gnss_has_new_data() override;
  bool gnss_read(rosflight_firmware::GNSSData *gnss, rosflight_firmware::GNSSFull *gnss_full) override;

  // RC
  void rc_init(rc_type_t rc_type) override;
  bool rc_has_new_data() override;
  bool rc_lost() override;
  //		float rc_read(uint8_t channel) override;
  float rc_read(uint8_t chan) override;

  // PWM
  void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override;
  void pwm_disable() override;
  void pwm_write(uint8_t channel, float value) override;
  uint32_t pwm_init_timers(uint32_t servo_pwm_period_us);

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

  // Backup Data
  void backup_memory_init() override;
  bool backup_memory_read(void *dest, size_t len) override;
  void backup_memory_write(const void *src, size_t len) override;
  void backup_memory_clear(size_t len) override;
};

#endif /* VARMINT_H_ */
