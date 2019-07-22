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


#include "breezy_board.h"

namespace rosflight_firmware
{

BreezyBoard::BreezyBoard() {}

void BreezyBoard::init_board()
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();
  _board_revision = 2;
}

void BreezyBoard::board_reset(bool bootloader)
{
  systemReset(bootloader);
}

// clock

uint32_t BreezyBoard::clock_millis()
{
  return millis();
}

uint64_t BreezyBoard::clock_micros()
{
  return micros();
}

void BreezyBoard::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial

void BreezyBoard::serial_init(uint32_t baud_rate, uint32_t dev)
{
  (void)dev;
  Serial1 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX);
}

void BreezyBoard::serial_write(const uint8_t *src, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    serialWrite(Serial1, src[i]);
  }
}

uint16_t BreezyBoard::serial_bytes_available()
{
  return serialTotalBytesWaiting(Serial1);
}

uint8_t BreezyBoard::serial_read()
{
  return serialRead(Serial1);
}

void BreezyBoard::serial_flush()
{
  return;
}

// sensors

void BreezyBoard::sensors_init()
{
  // Initialize I2c
  i2cInit(I2CDEV_2);

  while (millis() < 50);

  i2cWrite(0,0,0);
  if (bmp280_init())
    baro_type = BARO_BMP280;
  else if (ms5611_init())
    baro_type = BARO_MS5611;

  hmc5883lInit();
  mb1242_init();
  ms4525_init();


  // IMU
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &_gyro_scale, _board_revision);
  _accel_scale = 9.80665f/acc1G;
}

uint16_t BreezyBoard::num_sensor_errors()
{
  return i2cGetErrorCounter();
}

bool BreezyBoard::new_imu_data()
{
  return mpu6050_new_data();
}

bool BreezyBoard::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
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

void BreezyBoard::imu_not_responding_error()
{
  // If the IMU is not responding, then we need to change where we look for the interrupt
  _board_revision = (_board_revision < 4) ? 5 : 2;
  sensors_init();
}

void BreezyBoard::mag_read(float mag[3])
{
  // Convert to NED
  hmc5883l_async_read(mag);
}

bool BreezyBoard::mag_present()
{
  return hmc5883l_present();
}

void BreezyBoard::mag_update()
{
  hmc5883l_request_async_update();
}

void BreezyBoard::baro_update()
{
  if (baro_type == BARO_BMP280)
    bmp280_async_update();
  else if (baro_type == BARO_MS5611)
    ms5611_async_update();
  else
  {
    bmp280_async_update();
    ms5611_async_update();
  }
}



void BreezyBoard::baro_read(float *pressure, float *temperature)
{
  if (baro_type == BARO_BMP280)
  {
    bmp280_async_update();
    bmp280_async_read(pressure, temperature);
  }
  else if (baro_type == BARO_MS5611)
  {
    ms5611_async_update();
    ms5611_async_read(pressure, temperature);
  }
}

bool BreezyBoard::baro_present()
{
  if (baro_type == BARO_BMP280)
    return bmp280_present();
  else if (baro_type == BARO_MS5611)
    return ms5611_present();
  else
  {
    if (bmp280_present())
    {
      baro_type = BARO_BMP280;
      return true;
    }
    else if (ms5611_present())
    {
      baro_type = BARO_MS5611;
      return true;
    }
  }
  return false;
}

bool BreezyBoard::diff_pressure_present()
{
  return ms4525_present();
}

void BreezyBoard::diff_pressure_update()
{
  return ms4525_async_update();
}

void BreezyBoard::diff_pressure_read(float *diff_pressure, float *temperature)
{
  ms4525_async_update();
  ms4525_async_read(diff_pressure, temperature);
}

void BreezyBoard::sonar_update()
{
  if (sonar_type == SONAR_I2C || sonar_type == SONAR_NONE)
    mb1242_async_update();

  // We don't need to actively update the pwm sonar
}

bool BreezyBoard::sonar_present()
{
  if (sonar_type == SONAR_I2C)
    return mb1242_present();
  else if (sonar_type == SONAR_PWM)
    return sonarPresent();
  else
  {
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
  }
  return false;
}

float BreezyBoard::sonar_read()
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

uint16_t num_sensor_errors()
{
  return i2cGetErrorCounter();
}

// PWM

void BreezyBoard::rc_init(rc_type_t rc_type)
{
  (void) rc_type; // TODO SBUS is not supported on F1
  pwmInit(true, false, false, pwm_refresh_rate_, pwm_idle_pwm_);
}

void BreezyBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  pwm_refresh_rate_ = refresh_rate;
  pwm_idle_pwm_ = idle_pwm;
  pwmInit(true, false, false, pwm_refresh_rate_, pwm_idle_pwm_);
}

void BreezyBoard::pwm_disable()
{
  pwm_refresh_rate_ = 50;
  pwm_idle_pwm_ = 0;
  pwmInit(true, false, false, pwm_refresh_rate_, pwm_idle_pwm_);
}

float BreezyBoard::rc_read(uint8_t channel)
{
  return (float)(pwmRead(channel) - 1000)/1000.0;
}

void BreezyBoard::pwm_write(uint8_t channel, float value)
{
  pwmWriteMotor(channel, static_cast<uint16_t>(value * 1000) + 1000);
}

bool BreezyBoard::rc_lost()
{
  return ((millis() - pwmLastUpdate()) > 40);
}

// non-volatile memory

void BreezyBoard::memory_init()
{
  initEEPROM();
}

bool BreezyBoard::memory_read(void *dest, size_t len)
{
  return readEEPROM(dest, len);
}

bool BreezyBoard::memory_write(const void *src, size_t len)
{
  return writeEEPROM(src, len);
}

//GNSS is not supported on breezy boards
GNSSData BreezyBoard::gnss_read()
{
  return {};
}

//GNSS is not supported on breezy boards
GNSSRaw BreezyBoard::gnss_raw_read()
{
  return {};
}

//GNSS is not supported on breezy boards
bool BreezyBoard::gnss_has_new_data()
{
  return false;
}

// LED

void BreezyBoard::led0_on()
{
  LED0_ON;
}

void BreezyBoard::led0_off()
{
  LED0_OFF;
}

void BreezyBoard::led0_toggle()
{
  LED0_TOGGLE;
}

void BreezyBoard::led1_on()
{
  LED1_ON;
}

void BreezyBoard::led1_off()
{
  LED1_OFF;
}

void BreezyBoard::led1_toggle()
{
  LED1_TOGGLE;
}

bool BreezyBoard::has_backup_data()
{
  return false;
}

BackupData BreezyBoard::get_backup_data()
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  BackupData blank_data = {0};
#pragma GCC diagnostic pop
  return blank_data;
}

}

#pragma GCC diagnostic pop
