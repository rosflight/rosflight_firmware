/**
 ******************************************************************************
 * File     : Varmint.cpp
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
#include "Varmint.h"

#include "Time64.h"

#include <ctime>

Varmint varmint;
rosflight_firmware::Board & board = varmint;

extern Time64 time64;

////////////////////////////////////////////////////////////////////////////////////////
//
// Rosflight HAL for Varmint Board
//
////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
// board init

// Note: void Varmint::init_board(void) is in VarmintInit.cpp

///////////////////////////////////////////////////////////////////////////////////////////////
// board reset
void Varmint::board_reset(bool bootloader)
{
  pwm_disable();
  //	HAL_NVIC_SystemReset();
}

///////////////////////////////////////////////////////////////////////////////////////////////
// us clock
uint32_t Varmint::clock_millis() { return (time64.Us() ) / 1000; }
uint64_t Varmint::clock_micros() { return (time64.Us() ); }
void Varmint::clock_delay(uint32_t ms) { time64.dMs(ms); }

///////////////////////////////////////////////////////////////////////////////////////////////
// serial comms to the Companion computer
void Varmint::serial_init(uint32_t baud_rate, uint32_t dev)
{
  serial_device_ = USE_TELEM; // dev; // 1 = telem uart, otherwise = VCP
  if (serial_device_ == 1) telem_.reset_baud(baud_rate);
}

void Varmint::serial_write(const uint8_t * src, size_t len, uint8_t qos)
{
  SerialTxPacket p;
  p.header.timestamp = time64.Us();
  p.payloadSize = len;
  p.qos = qos;

  if (len > (256 + 8)) len = (256 + 8);
  memcpy(p.payload, src, len);

  if (serial_device_ == 1) telem_.writePacket(&p);
  else vcp_.writePacket(&p);
}

uint16_t Varmint::serial_bytes_available(void)
{
  if (serial_device_ == 1) return telem_.byteCount();
  else return vcp_.byteCount();
}

uint8_t Varmint::serial_read(void)
{
  uint8_t c = 0;
  if (serial_device_ == 1) telem_.readByte(&c);
  else vcp_.readByte(&c);
  return c;
}

void Varmint::serial_flush(void)
{
  // do nothing
}

///////////////////////////////////////////////////////////////////////////////////////////////
//
// Sensors
//
//////////////////////////////////////////////////////////////////////////////////////////////////

void Varmint::sensors_init()
{
  sensor_errors_ = 0;
  for (uint32_t i = 0; i < varmint.status_len(); i++) {
    if (varmint.status(i)->status() != DRIVER_OK) sensor_errors_++;
  }
}
uint16_t Varmint::sensors_errors_count() { return sensor_errors_; }

uint16_t Varmint::sensors_init_message_count() { return varmint.status_len(); }

bool Varmint::sensors_init_message_good(uint16_t i) { return varmint.status(i)->initGood(); }

uint16_t Varmint::sensors_init_message(char * message, uint16_t size, uint16_t i)
{
  if (i > varmint.status_len()) return 0;

  uint32_t status = varmint.status(i)->status();
  if (status == DRIVER_OK) {
    snprintf(message, size, "%s: INIT OK", varmint.status(i)->name());
  } else { //PTT TODO: we can add better messages later
    snprintf(message, size, "%s: INIT ERROR 0x%08lX", varmint.status(i)->name(), status);
  }
  return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// IMU
bool Varmint::imu_read(rosflight_firmware::ImuStruct * imu)
{
  return imu0_.read((uint8_t *) imu, sizeof(rosflight_firmware::ImuStruct));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// MAG
bool Varmint::mag_read(rosflight_firmware::MagStruct * mag)
{
  return mag_.read((uint8_t *) mag, sizeof(rosflight_firmware::MagStruct));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Baro
bool Varmint::baro_read(rosflight_firmware::PressureStruct * baro)
{
  return baro_.read((uint8_t *) baro, sizeof(rosflight_firmware::PressureStruct));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Pitot
bool Varmint::diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure)
{
  return pitot_.read((uint8_t *) diff_pressure, sizeof(rosflight_firmware::PressureStruct));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Sonar
bool Varmint::range_read(rosflight_firmware::RangeStruct * range)
{
  (void) range; // unused
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Battery
bool Varmint::battery_read(rosflight_firmware::BatteryStruct * batt)
{
  AdcPacket p;
  if (adc_.read((uint8_t *) &p, sizeof(p))) {
    batt->header = p.header;
    batt->current = p.volts[ADC_BATTERY_CURRENT];
    batt->voltage = p.volts[ADC_BATTERY_VOLTS];
    batt->temperature = p.temperature;
    return true;
  }
  return false;
}
void Varmint::battery_voltage_set_multiplier(double multiplier)
{
  if (multiplier == 0) return;
  adc_.setScaleFactor(ADC_BATTERY_VOLTS, multiplier);
}
void Varmint::battery_current_set_multiplier(double multiplier)
{
  if (multiplier == 0) return;
  adc_.setScaleFactor(ADC_BATTERY_CURRENT, multiplier);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// GNSS
bool Varmint::gnss_read(rosflight_firmware::GnssStruct * gnss)
{
  return gps_->read((uint8_t *) gnss, sizeof(rosflight_firmware::GnssStruct));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// RC
void Varmint::rc_init(rc_type_t rc_type) { (void) rc_type; };
bool Varmint::rc_read(rosflight_firmware::RcStruct * rc_struct)
{
  return rc_.read((uint8_t *) rc_struct, sizeof(rosflight_firmware::RcStruct));
}

///////////////////////////////////////////////////////////////////////////////////////////////
// PWM

void Varmint::pwm_init(const float * rate, uint32_t channels) { pwm_.updateConfig(rate, channels); }
void Varmint::pwm_disable(void)
{
  for (int ch = 0; ch < PWM_CHANNELS; ch++) pwm_.disable(ch);
}
void Varmint::pwm_write(float * value, uint32_t channels) { pwm_.write(value, channels); }

///////////////////////////////////////////////////////////////////////////////////////////////
// LEDs
void Varmint::led0_on() { RED_HI; }
void Varmint::led0_off() { RED_LO; }
void Varmint::led0_toggle() { RED_TOG; }

void Varmint::led1_on() { BLU_HI; }
void Varmint::led1_off() { BLU_LO; }
void Varmint::led1_toggle() { BLU_TOG; }

///////////////////////////////////////////////////////////////////////////////////////////////
// Backup Data (Register and SRAM)
// https://stackoverflow.com/questions/20667754/how-to-use-backup-sram-as-eeprom-in-stm32f4
// from \Drivers\CMSIS\Device\ST\STM32H7xx\Include\stm32h753xx.h(2141)
//#define D3_BKPSRAM_BASE           (0x38800000UL) /*!< Base address of : Backup SRAM(4 KB) over AXI->AHB Bridge */
//#define D3_SRAM_BASE              (0x38000000UL) /*!< Base address of : Backup SRAM(64 KB) over AXI->AHB Bridge

#define D3_BKPSRAM_BASE_LEN (4096U)
#define D3_SRAM_BASE_LEN (65536U)
void Varmint::backup_memory_init() {}
bool Varmint::backup_memory_read(void * dest, size_t len)
{
  //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
  //	HAL_PWR_EnableBkUpAccess();
  //	memcpy(dest, (void*)D3_BKPSRAM_BASE, len);
  //	HAL_PWR_DisableBkUpAccess();
  return true;
}
void Varmint::backup_memory_write(const void * src, size_t len)
{
  //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
  //	HAL_PWR_EnableBkUpAccess();
  //	memcpy((void*)D3_BKPSRAM_BASE, src, len);
  //	HAL_PWR_DisableBkUpAccess();
}
void Varmint::backup_memory_clear(size_t len)
{
  //	HAL_PWR_EnableBkUpAccess();
  //	memset((void*)D3_BKPSRAM_BASE, 0, D3_BKPSRAM_BASE_LEN);
  //	HAL_PWR_DisableBkUpAccess();
}

void Varmint::memory_init() {} // do nothing

bool Varmint::memory_read(void * dest, size_t len) { return sd_.read((uint8_t *) dest, len); }
bool Varmint::memory_write(const void * src, size_t len) { return sd_.write((uint8_t *) src, len); }

///////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions (not part of parent class)
