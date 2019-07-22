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

#include "airbourne_board.h"

namespace rosflight_firmware
{

AirbourneBoard::AirbourneBoard()
{
}

void AirbourneBoard::init_board()
{
  systemInit();
  led2_.init(LED2_GPIO, LED2_PIN);
  led1_.init(LED1_GPIO, LED1_PIN);

  int_i2c_.init(&i2c_config[BARO_I2C]);
  ext_i2c_.init(&i2c_config[EXTERNAL_I2C]);
  spi1_.init(&spi_config[MPU6000_SPI]);
  spi3_.init(&spi_config[FLASH_SPI]);
  uart1_.init(&uart_config[UART1], 115200, UART::MODE_8N1);
  uart3_.init(&uart_config[UART3], 115200, UART::MODE_8N1);

  backup_sram_init();

  current_serial_ = &vcp_;    //uncomment this to switch to VCP as the main output
}

void AirbourneBoard::board_reset(bool bootloader)
{
  (void)bootloader;
  NVIC_SystemReset();
}

// clock
uint32_t AirbourneBoard::clock_millis()
{
  return millis();
}

uint64_t AirbourneBoard::clock_micros()
{
  return micros();
}

void AirbourneBoard::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial
void AirbourneBoard::serial_init(uint32_t baud_rate, uint32_t dev)
{
  vcp_.init();
  switch (dev)
  {
  case SERIAL_DEVICE_UART3:
    uart3_.init(&uart_config[UART3], baud_rate);
    current_serial_ = &uart3_;
    secondary_serial_device_ = SERIAL_DEVICE_UART3;
    break;
  default:
    current_serial_ = &vcp_;
    secondary_serial_device_ = SERIAL_DEVICE_VCP;
  }
}

void AirbourneBoard::serial_write(const uint8_t *src, size_t len)
{
  current_serial_->write(src, len);
}

uint16_t AirbourneBoard::serial_bytes_available()
{
  if (vcp_.connected() || secondary_serial_device_ == SERIAL_DEVICE_VCP)
  {
    current_serial_ = &vcp_;
  }
  else
  {
    switch (secondary_serial_device_)
    {
    case SERIAL_DEVICE_UART3:
      current_serial_ = &uart3_;
      break;
    default:
      // no secondary serial device
      break;
    }
  }

  return current_serial_->rx_bytes_waiting();
}

uint8_t AirbourneBoard::serial_read()
{

  return current_serial_->read_byte();
}

void AirbourneBoard::serial_flush()
{
  current_serial_->flush();
}


// sensors
void AirbourneBoard::sensors_init()
{
  while (millis() < 50) {} // wait for sensors to boot up
  imu_.init(&spi1_);

  baro_.init(&int_i2c_);
  mag_.init(&int_i2c_);
  sonar_.init(&ext_i2c_);
  airspeed_.init(&ext_i2c_);
  // gnss_.init(&uart1_);
}

uint16_t AirbourneBoard::num_sensor_errors()
{
  return ext_i2c_.num_errors();
}

bool AirbourneBoard::new_imu_data()
{
  return imu_.new_data();
}

bool AirbourneBoard::imu_read(float accel[3], float *temperature, float gyro[3], uint64_t *time_us)
{
  float read_accel[3], read_gyro[3];
  imu_.read(read_accel, read_gyro, temperature, time_us);

  accel[0] = -read_accel[1];
  accel[1] = -read_accel[0];
  accel[2] = -read_accel[2];

  gyro[0] = -read_gyro[1];
  gyro[1] = -read_gyro[0];
  gyro[2] = -read_gyro[2];

  return true;
}

void AirbourneBoard::imu_not_responding_error()
{
  sensors_init();
}

bool AirbourneBoard::mag_present()
{
  mag_.update();
  return mag_.present();
}

void AirbourneBoard::mag_update()
{
  mag_.update();
}

void AirbourneBoard::mag_read(float mag[3])
{
  mag_.update();
  mag_.read(mag);
}
bool AirbourneBoard::baro_present()
{
  baro_.update();
  return baro_.present();
}

void AirbourneBoard::baro_update()
{
  baro_.update();
}

void AirbourneBoard::baro_read(float *pressure, float *temperature)
{
  baro_.update();
  baro_.read(pressure, temperature);
}

bool AirbourneBoard::diff_pressure_present()
{
  return airspeed_.present();
}

void AirbourneBoard::diff_pressure_update()
{
  airspeed_.update();
}


void AirbourneBoard::diff_pressure_read(float *diff_pressure, float *temperature)
{
  (void) diff_pressure;
  (void) temperature;
  airspeed_.update();
  airspeed_.read(diff_pressure, temperature);
}

bool AirbourneBoard::sonar_present()
{
  return sonar_.present();
}

void AirbourneBoard::sonar_update()
{
  sonar_.update();
}

float AirbourneBoard::sonar_read()
{
  return sonar_.read();
}

bool AirbourneBoard::gnss_present()
{
  // return gnss_.present();
  return false;
}
void AirbourneBoard::gnss_update() {}
bool AirbourneBoard::gnss_has_new_data()
{
  // return this->gnss_.new_data();
  return false;
}
//This method translates the UBLOX driver interface into the ROSFlight interface
//If not gnss_has_new_data(), then this may return 0's for ECEF position data,
//ECEF velocity data, or both
GNSSData AirbourneBoard::gnss_read()
{
  // UBLOX::GNSSPVT gnss_pvt= gnss_.read();
  // UBLOX::GNSSPosECEF pos_ecef = gnss_.read_pos_ecef();
  // UBLOX::GNSSVelECEF vel_ecef = gnss_.read_vel_ecef();
  GNSSData gnss = {};
  // gnss.time_of_week = gnss_pvt.time_of_week;
  // gnss.time = gnss_pvt.time;
  // gnss.nanos = gnss_pvt.nanos;
  // gnss.lat = gnss_pvt.lat;
  // gnss.lon = gnss_pvt.lon;
  // gnss.height = gnss_pvt.height;
  // gnss.vel_n = gnss_pvt.vel_n;
  // gnss.vel_e = gnss_pvt.vel_e;
  // gnss.vel_d = gnss_pvt.vel_d;
  // gnss.h_acc = gnss_pvt.h_acc;
  // gnss.v_acc = gnss_pvt.v_acc;
  // //Does not include ECEF position data if the timestamp doesn't match
  // //See UBLOX::new_data() for reasoning
  // if (gnss.time_of_week == pos_ecef.time_of_week)
  // {
  //   gnss.ecef.x = pos_ecef.x;
  //   gnss.ecef.y = pos_ecef.y;
  //   gnss.ecef.z = pos_ecef.z;
  //   gnss.ecef.p_acc = pos_ecef.p_acc;
  // }
  // //Does not include ECEF position data if the timestamp doesn't match
  // //See UBLOX::new_data() for reasoning
  // if (gnss.time_of_week == vel_ecef.time_of_week)
  // {
  //   gnss.ecef.vx = vel_ecef.vx;
  //   gnss.ecef.vy = vel_ecef.vy;
  //   gnss.ecef.vz = vel_ecef.vz;
  //   gnss.ecef.s_acc = vel_ecef.s_acc;
  // }

  return gnss;
}
GNSSRaw AirbourneBoard::gnss_raw_read()
{
//  UBLOX::NAV_PVT_t pvt = gnss_.read_raw();
  GNSSRaw raw = {};
  // raw.time_of_week = pvt.iTOW;
  // raw.year = pvt.time.year;
  // raw.month = pvt.time.month;
  // raw.day = pvt.time.day;
  // raw.hour = pvt.time.hour;
  // raw.min = pvt.time.min;
  // raw.sec = pvt.time.sec;
  // raw.valid = pvt.time.valid;
  // raw.t_acc = pvt.time.tAcc;
  // raw.nano = pvt.time.nano;
  // raw.fix_type = pvt.fixType;
  // raw.num_sat = pvt.numSV;
  // raw.lon = pvt.lon;
  // raw.lat = pvt.lat;
  // raw.height = pvt.height;
  // raw.height_msl = pvt.hMSL;
  // raw.h_acc = pvt.hAcc;
  // raw.v_acc = pvt.vAcc;
  // raw.vel_n = pvt.velN;
  // raw.vel_e = pvt.velE;
  // raw.vel_d = pvt.velD;
  // raw.g_speed = pvt.gSpeed;
  // raw.head_mot = pvt.headMot;
  // raw.s_acc = pvt.sAcc;
  // raw.head_acc = pvt.headAcc;
  // raw.p_dop = pvt.pDOP;
  // raw.rosflight_timestamp = gnss_.get_last_pvt_timestamp();
  return raw;
}

// PWM
void AirbourneBoard::rc_init(rc_type_t rc_type)
{
  switch (rc_type)
  {
  case RC_TYPE_SBUS:
    uart1_.init(&uart_config[UART1], 100000, UART::MODE_8E2);
    inv_pin_.init(SBUS_INV_GPIO, SBUS_INV_PIN, GPIO::OUTPUT);
    rc_sbus_.init(&inv_pin_, &uart1_);
    rc_ = &rc_sbus_;
    break;
  case RC_TYPE_PPM:
  default:
    rc_ppm_.init(&pwm_config[RC_PPM_PIN]);
    rc_ = &rc_ppm_;
    break;
  }
}

float AirbourneBoard::rc_read(uint8_t channel)
{
  return rc_->read(channel);
}

void AirbourneBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].init(&pwm_config[i], refresh_rate, 2000, 1000);
    esc_out_[i].writeUs(idle_pwm);
  }
}

void AirbourneBoard::pwm_disable()
{
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].disable();
  }
}

void AirbourneBoard::pwm_write(uint8_t channel, float value)
{
  if (channel < PWM_NUM_OUTPUTS)
  {
    esc_out_[channel].write(value);
  }
}

bool AirbourneBoard::rc_lost()
{
  return rc_->lost();
}

// non-volatile memory
void AirbourneBoard::memory_init()
{
  return flash_.init(&spi3_);
}

bool AirbourneBoard::memory_read(void *data, size_t len)
{
  return flash_.read_config(reinterpret_cast<uint8_t *>(data), len);
}

bool AirbourneBoard::memory_write(const void *data, size_t len)
{
  return flash_.write_config(reinterpret_cast<const uint8_t *>(data), len);
}

// LED
void AirbourneBoard::led0_on()
{
  led1_.on();
}

void AirbourneBoard::led0_off()
{
  led1_.off();
}

void AirbourneBoard::led0_toggle()
{
  led1_.toggle();
}

void AirbourneBoard::led1_on()
{
  led2_.on();
}

void AirbourneBoard::led1_off()
{
  led2_.off();
}

void AirbourneBoard::led1_toggle()
{
  led2_.toggle();
}

//Backup memory
bool AirbourneBoard::has_backup_data()
{
  BackupData backup_data = backup_sram_read();
  return (check_backup_checksum(backup_data) && backup_data.error_code!=0);
}

rosflight_firmware::BackupData AirbourneBoard::get_backup_data()
{
  return backup_sram_read();
}

} // namespace rosflight_firmware
