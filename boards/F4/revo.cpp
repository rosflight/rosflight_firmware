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
}
Revo::~Revo()
{

}

void Revo::init_board(void)
{
  systemInit();
  //led2_.init(LED2_GPIO, LED2_PIN);
  //led1_.init(LED1_GPIO, LED1_PIN);

  //int_i2c_.init(&i2c_config[MAG_I2C]);
  //ext_i2c_.init(&i2c_config[EXTERNAL_I2C]);
  //spi1_.init(&spi_config[MPU6000_SPI]);
  //spi3_.init(&spi_config[FLASH_SPI]);

  //serial_interfaces_[0]=&//vcp_;
  //serial_interfaces_[1]=&uart_;

  this->current_serial_=&uart_;
  //this->current_serial_=&//vcp_;    //uncomment this to switch to VCP
}

void Revo::board_reset(bool bootloader)
{
  NVIC_SystemReset();
}

// clock
uint32_t Revo::clock_millis()
{
  return millis();
}

uint64_t Revo::clock_micros()
{
  return micros();
}

void Revo::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial
void Revo::serial_init(uint32_t baud_rate)
{
  uart_.init(&uart_config[2], 115200);
  uint8_t hello[6]="hello";
  //vcp_.set_baud_rate(baud_rate);
  //vcp_.init();
  uint8_t message[6]="init\n";
  //vcp_.write(message,5);
  uart_.write(hello,6);
//  //vcp_.write(hello,6);
//  //vcp_.flush();
  delay(200);
  //uart_.flush();

}

void Revo::serial_write(const uint8_t *src, size_t len)
{

    uint8_t message[7]="write(";
    uint8_t message2[6]=") \n\n\n";
    //vcp_.write(message,6);
    //vcp_.write(src,len);
    //vcp_.write(message2,5);
  current_serial_->write(src, len);
  volatile uint8_t value=*(src+1);
  //delay(200);
  //For testing only
  //vcp_.write(src,len);
}

uint16_t Revo::serial_bytes_available(void)
{
    uint8_t message[7]="avail\n";
    //vcp_.write(message,6);
  return current_serial_->rx_bytes_waiting();
}

uint8_t Revo::serial_read(void)
{

    uint8_t message[6]="read\n";
    //vcp_.write(message,5);
  //For testing only
  uint8_t byte=current_serial_->read_byte();
  //vcp_.write(&byte,1);
  return byte;
  //return current_serial_->read_byte();
}

void Revo::serial_flush()
{

    uint8_t message[7]="flush\n";
    //vcp_.write(message,6);
  current_serial_->flush();
}

Serial** Revo::get_serial_interfaces()
{
    return serial_interfaces_;
}

uint8_t Revo::get_serial_count()
{
    return 2;
}


// sensors
void Revo::sensors_init()
{
  imu_.init(&spi1_);
  mag_.init(&int_i2c_);
  baro_.init(&int_i2c_);
  airspeed_.init(&ext_i2c_);

  while(millis() < 50); // wait for sensors to boot up
}

uint16_t Revo::num_sensor_errors(void)
{
  return int_i2c_.num_errors();
}

bool Revo::new_imu_data()
{
  return imu_.new_data();
}

bool Revo::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
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

void Revo::imu_not_responding_error(void)
{
  sensors_init();
}

void Revo::mag_read(float mag[3])
{
  mag_.update();
  mag_.read(mag);
}

bool Revo::mag_check(void)
{
  mag_.update();
  return mag_.present();
}

void Revo::baro_read(float *pressure, float *temperature)
{
  baro_.update();
  baro_.read(pressure, temperature);
}

bool Revo::baro_check()
{
  baro_.update();
  return baro_.present();
}

bool Revo::diff_pressure_check(void)
{
  airspeed_.update();
  return airspeed_.present();
}

void Revo::diff_pressure_read(float *diff_pressure, float *temperature)
{
  airspeed_.update();
  airspeed_.read(diff_pressure, temperature);
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
void Revo::rc_init(rc_type_t rc_type)
{
    //Testing
  /*switch (rc_type)
  {
  case RC_TYPE_SBUS:
    sbus_uart_.init(&uart_config[0], 100000, UART::MODE_8E2);
    inv_pin_.init(SBUS_INV_GPIO, SBUS_INV_PIN, GPIO::OUTPUT);
    rc_sbus_.init(&inv_pin_, &sbus_uart_);
    rc_ = &rc_sbus_;
    break;
  case RC_TYPE_PPM:
    rc_ppm_.init(&pwm_config[RC_PPM_PIN]);
    rc_ = &rc_ppm_;
    break;
  }*/
    rc_ppm_.init(&pwm_config[RC_PPM_PIN]);
    rc_ = &rc_ppm_;
    //END TESTING
}

float Revo::rc_read(uint8_t channel)
{
  return rc_->read(channel);
}

void Revo::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].init(&pwm_config[i], refresh_rate, 2000, 1000);
    esc_out_[i].writeUs(idle_pwm);
  }
}

void Revo::pwm_write(uint8_t channel, float value)
{
  esc_out_[channel].write(value);
}

bool Revo::rc_lost()
{
  return rc_->lost();
}

// non-volatile memory
void Revo::memory_init(void)
{
  return flash_.init(&spi3_);
}

bool Revo::memory_read(void * data, size_t len)
{
  return flash_.read_config((uint8_t*)data, len);
}

bool Revo::memory_write(const void * data, size_t len)
{
  return flash_.write_config((uint8_t*)data, len);
}

// LED
void Revo::led0_on(void) { led1_.on(); }
void Revo::led0_off(void) { led1_.off(); }
void Revo::led0_toggle(void) { led1_.toggle(); }

void Revo::led1_on(void) { led2_.on(); }
void Revo::led1_off(void) { led2_.off(); }
void Revo::led1_toggle(void) { led2_.toggle(); }
}

#pragma GCC diagnostic pop
