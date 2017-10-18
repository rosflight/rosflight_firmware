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

extern "C"
{

#include <breezystm32.h>
#include "flash.h"
extern void SetSysClock(bool overclock);

}


#include "naze32.h"

namespace rosflight_firmware {

Naze32::Naze32(){}

void Naze32::init_board(void)
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();
  _board_revision = 2;
}

void Naze32::board_reset(bool bootloader)
{
  systemReset(bootloader);
}

// clock

uint32_t Naze32::clock_millis()
{
  return millis();
}

uint64_t Naze32::clock_micros()
{
  return micros();
}

void Naze32::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial

void Naze32::serial_init(uint32_t baud_rate)
{
  Serial1 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX);
}

void Naze32::serial_write(const uint8_t *src, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    serialWrite(Serial1, src[i]);
  }
}

uint16_t Naze32::serial_bytes_available(void)
{
  return serialTotalBytesWaiting(Serial1);
}

uint8_t Naze32::serial_read(void)
{
  return serialRead(Serial1);
}

// sensors

void Naze32::sensors_init()
{
  // Initialize I2c
  i2cInit(I2CDEV_2);

  while(millis() < 50);

  i2cWrite(0,0,0);
  ms5611_init();
  hmc5883lInit(_board_revision);
  mb1242_init();
  ms4525_init();

  // IMU
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &_gyro_scale, _board_revision);
  _accel_scale = 9.80665f/acc1G;
}

uint16_t Naze32::num_sensor_errors(void)
{
  return i2cGetErrorCounter();
}

bool Naze32::new_imu_data()
{
  return mpu6050_new_data();
}

bool Naze32::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  volatile int16_t gyro_raw[3], accel_raw[3];
  volatile int16_t raw_temp;
  mpu6050_async_read_all(accel_raw, &raw_temp, gyro_raw, time_us);

  accel[0] = accel_raw[0] * _accel_scale;
  accel[1] = -accel_raw[1] * _accel_scale;
  accel[2] = -accel_raw[2] * _accel_scale;

  gyro[0] = gyro_raw[0] * _gyro_scale;
  gyro[1] = -gyro_raw[1] * _gyro_scale;
  gyro[2] = -gyro_raw[2] * _gyro_scale;

  (*temperature) = (float)raw_temp/340.0f + 36.53f;

  if (accel[0] == 0 && accel[1] == 0 && accel[2] == 0)
  {
    return false;
  }
  else return true;
}

void Naze32::imu_not_responding_error(void)
{
  // If the IMU is not responding, then we need to change where we look for the interrupt
  _board_revision = (_board_revision < 4) ? 5 : 2;
  sensors_init();
}

void Naze32::mag_read(float mag[3])
{
  // Convert to NED
  int16_t raw_mag[3];
  //  hmc5883l_update();
  hmc5883l_request_async_update();
  hmc5883l_async_read(raw_mag);
  mag[0] = (float)raw_mag[0];
  mag[1] = (float)raw_mag[1];
  mag[2] = (float)raw_mag[2];
}

bool Naze32::mag_check(void)
{
  return hmc5883l_present();
}

void Naze32::baro_read(float *pressure, float *temperature)
{
  ms5611_async_update();
  ms5611_async_read(pressure, temperature);
}

bool Naze32::baro_check()
{
  ms5611_async_update();
  return ms5611_present();
}

bool Naze32::diff_pressure_check(void)
{
  ms4525_async_update();
  return ms4525_present();
}

void Naze32::diff_pressure_read(float *diff_pressure, float *temperature)
{
  ms4525_async_update();
  ms4525_async_read(diff_pressure, temperature);
}

bool Naze32::sonar_check(void)
{
  mb1242_async_update();
  if (mb1242_present())
  {
    sonar_type = SONAR_I2C;
    return true;
  }
  else if (sonarPresent())
  {
    sonar_type = SONAR_PWM;
    return true;
  }
  else
  {
    sonar_type = SONAR_NONE;
    return false;
  }
}

float Naze32::sonar_read(void)
{
  if (sonar_type == SONAR_I2C)
  {
    mb1242_async_update();
    return mb1242_async_read();
  }
  else if (sonar_type == SONAR_PWM)
    return sonarRead(6);
  else
    return 0.0f;
}

uint16_t num_sensor_errors(void)
{
  return i2cGetErrorCounter();
}

// PWM

void Naze32::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t Naze32::pwm_read(uint8_t channel)
{
  return pwmRead(channel);
}

void Naze32::pwm_write(uint8_t channel, uint16_t value)
{
  pwmWriteMotor(channel, value);
}

bool Naze32::pwm_lost()
{
  return ((millis() - pwmLastUpdate()) > 40);
}

// non-volatile memory

void Naze32::memory_init(void)
{
  initEEPROM();
}

bool Naze32::memory_read(void * dest, size_t len)
{
  return readEEPROM(dest, len);
}

bool Naze32::memory_write(const void * src, size_t len)
{
  return writeEEPROM(src, len);
}

// LED

void Naze32::led0_on(void) { LED0_ON; }
void Naze32::led0_off(void) { LED0_OFF; }
void Naze32::led0_toggle(void) { LED0_TOGGLE; }

void Naze32::led1_on(void) { LED1_ON; }
void Naze32::led1_off(void) { LED1_OFF; }
void Naze32::led1_toggle(void) { LED1_TOGGLE; }

}

#pragma GCC diagnostic pop
