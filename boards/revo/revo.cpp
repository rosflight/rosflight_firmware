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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"

#include "revo.h"

namespace rosflight_firmware {

Revo::Revo()
{
  volatile int debug = 1;
}

void Revo::init_board(void)
{
  systemInit();
//  warn_.init(LED1_GPIO, LED1_PIN);
//  info_.init(LED2_GPIO, LED2_PIN);
}

void Revo::board_reset(bool bootloader)
{
//  systemReset(bootloader);
}

// clock

uint32_t Revo::clock_millis()
{
  return 0;
}

uint64_t Revo::clock_micros()
{
  return 0;
}

void Revo::clock_delay(uint32_t milliseconds)
{
//  delay(milliseconds);
}

// serial
void Revo::serial_init(uint32_t baud_rate)
{
}

void Revo::serial_write(const uint8_t *src, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
//    vcp_.write(src, len);
  }
}

uint16_t Revo::serial_bytes_available(void)
{
//  return vcp_.rx_bytes_waiting();
  return 0;
}

uint8_t Revo::serial_read(void)
{
//  return vcp_.read_byte();
  return 0;
}

// sensors

void Revo::sensors_init()
{
//  i2c_.init(I2C1);
//  spi_.init(SPI1);
//  imu_.init(&spi_);
//  mag_.init(&i2c_);
//  baro_.init(&i2c_);
//  while(millis() < 50);
//  i2c_.write(0,0,0);
}

uint16_t Revo::num_sensor_errors(void)
{
//  return i2c_.num_errors();
  return 0;
}

bool Revo::new_imu_data()
{
//  return imu_.new_data();
  return 0;
}

bool Revo::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
//  imu_.read(accel, gyro, temperature, time_us);

//  accel[0] = accel[0];
//  accel[1] = -accel[1];
//  accel[2] = -accel[2];

//  gyro[0] = gyro[0];
//  gyro[1] = -gyro[1];
//  gyro[2] = -gyro[2];

  return true;
}

void Revo::imu_not_responding_error(void)
{
  // If the IMU is not responding, then we need to change where we look for the interrupt
  sensors_init();
}

void Revo::mag_read(float mag[3])
{
//  mag_.update();
//  mag_.read(mag);
}

bool Revo::mag_check(void)
{
//  return mag_.present();
  return false;
}

void Revo::baro_read(float *pressure, float *temperature)
{
//  baro_.update();
//  baro_.read(pressure, temperature);
}

bool Revo::baro_check()
{
//  baro_.update();
//  return baro_.present();
  return false;
}

bool Revo::diff_pressure_check(void)
{
  return false;
}

void Revo::diff_pressure_read(float *diff_pressure, float *temperature)
{
  return;
}

bool Revo::sonar_check(void)
{
  return false;
}

float Revo::sonar_read(void)
{
  return 0.0;
}

// PWM

void Revo::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
//  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
//  {
//    esc_out_[i].init(&pwm_config[i], refresh_rate, PWM_MAX_US, PWM_MIN_US);
//    esc_out_[i].writeUs(idle_pwm);
//  }
}

uint16_t Revo::pwm_read(uint8_t channel)
{
//  return rc_.read(channel);
  return 0;
}

void Revo::pwm_write(uint8_t channel, uint16_t value)
{
//  esc_out_[channel].writeUs(value);
}

bool Revo::pwm_lost()
{
//  return rc_.lost();
  return 0;
}

// non-volatile memory

void Revo::memory_init(void)
{
}

bool Revo::memory_read(void * dest, size_t len)
{
  return false;
}

bool Revo::memory_write(const void * src, size_t len)
{
  return false;
}

// LED

//void Revo::led0_on(void) { info_.on(); }
//void Revo::led0_off(void) { info_.off(); }
//void Revo::led0_toggle(void) { info_.toggle(); }

//void Revo::led1_on(void) { warn_.on(); }
//void Revo::led1_off(void) { warn_.off(); }
//void Revo::led1_toggle(void) { warn_.toggle(); }

void Revo::led0_on(void) { /*info_.on();*/ }
void Revo::led0_off(void) { /*info_.off();*/ }
void Revo::led0_toggle(void) { /*info_.toggle();*/ }

void Revo::led1_on(void) { /*warn_.on();*/ }
void Revo::led1_off(void) {/* warn_.off();*/ }
void Revo::led1_toggle(void) { /*warn_.toggle();*/ }

}

#pragma GCC diagnostic pop
