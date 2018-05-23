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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <revo_f4.h>

#include "vcp.h"
#include "i2c.h"
#include "spi.h"
#include "mpu6000.h"
#include "ms5611.h"
#include "M25P16.h"
#include "hmc5883l.h"
#include "ms4525.h"
#include "rc_base.h"
#include "rc_ppm.h"
#include "rc_sbus.h"
#include "pwm.h"
#include "led.h"
#include "uart.h"

#include "board.h"

namespace rosflight_firmware {

class AirbourneBoard : public Board
{

private:
    VCP vcp_;
    I2C int_i2c_;
    I2C ext_i2c_;
    SPI spi1_;
    SPI spi3_;
    MPU6000 imu_;
    HMC5883L mag_;
    MS5611 baro_;
    MS4525 airspeed_;
    RC_SBUS rc_sbus_;
    UART sbus_uart_;
    GPIO inv_pin_;
    RC_PPM rc_ppm_;
    PWM_OUT esc_out_[PWM_NUM_OUTPUTS];
    LED led2_;
    LED led1_;
    M25P16 flash_;

    RC_BASE* rc_ = nullptr;

    std::function<void(void)> imu_callback_;

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



public:
  AirbourneBoard();

  bool new_imu_data_;
  uint64_t imu_time_us_;

  // setup
  void init_board(void) override;
  void board_reset(bool bootloader) override;

  // clock
  uint32_t clock_millis() override;
  uint64_t clock_micros() override;
  void clock_delay(uint32_t milliseconds) override;

  // serial
  void serial_init(uint32_t baud_rate) override;
  void serial_write(const uint8_t *src, size_t len) override;
  uint16_t serial_bytes_available(void) override;
  uint8_t serial_read(void) override;
  void serial_flush(void) override;

  // sensors
  void sensors_init() override;
  uint16_t num_sensor_errors(void) override;

  bool new_imu_data() override;
  bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
  void imu_not_responding_error() override;

  bool mag_present(void) override;
  void mag_update(void) override;
  void mag_read(float mag[3]) override;

  bool baro_present() override;
  void baro_update() override;
  void baro_read(float *pressure, float *temperature) override;

  bool diff_pressure_present(void) override;
  void diff_pressure_update(void) override;
  void diff_pressure_read(float *diff_pressure, float *temperature) override;

  bool sonar_present(void) override;
  void sonar_update(void) override;
  float sonar_read(void) override;

  // RC
  void rc_init(rc_type_t rc_type) override;
  bool rc_lost() override;
  float rc_read(uint8_t channel) override;

  // PWM
  void pwm_init(uint32_t refresh_rate, uint16_t  idle_pwm) override;
  void pwm_write(uint8_t channel, float value) override;

  // non-volatile memory
  void memory_init(void) override;
  bool memory_read(void * dest, size_t len) override;
  bool memory_write(const void *src, size_t len) override;

  // LEDs
  void led0_on(void) override;
  void led0_off(void) override;
  void led0_toggle(void) override;

  void led1_on(void) override;
  void led1_off(void) override;
  void led1_toggle(void) override;
};

}
