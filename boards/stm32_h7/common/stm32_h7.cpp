/**
 ******************************************************************************
 * File     : stm32_h7.cpp
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
#include "stm32_h7.hpp"

#include "Time64.h"

#include <ctime>

STM32H7Board stm32_h7_board;
rosflight_firmware::Board & board = stm32_h7_board;

extern Time64 time64;

////////////////////////////////////////////////////////////////////////////////////////
//
// Rosflight HAL for STM32H7 Board
//
////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
// board init

// Note: void STM32H7::init_board(void) is in STM32H7_Init.cpp

///////////////////////////////////////////////////////////////////////////////////////////////
// board reset
void STM32H7Board::board_reset(bool bootloader)
{
  pwm_disable();
  //	HAL_NVIC_SystemReset();
}

void STM32H7Board::clear_poll_clients() { poll_client_len_ = 0; }
void STM32H7Board::clear_exti_clients() { exti_client_len_ = 0; }
void STM32H7Board::clear_spi_clients() { spi_client_len_ = 0; }
void STM32H7Board::clear_i2c_clients() { i2c_client_len_ = 0; }
void STM32H7Board::clear_adc_clients() { adc_client_len_ = 0; }
void STM32H7Board::clear_cdc_clients() { cdc_client_len_ = 0; }
void STM32H7Board::clear_sd_clients() { sd_client_len_ = 0; }
void STM32H7Board::clear_uart_rxcplt_clients() { uart_rxcplt_client_len_ = 0; }
void STM32H7Board::clear_uart_rxisr_clients() { uart_rxisr_client_len_ = 0; }
void STM32H7Board::clear_uart_txcplt_clients() { uart_txcplt_client_len_ = 0; }

void STM32H7Board::register_poll_client(
  void * context, void (*callback)(void * context, uint64_t poll_counter), int32_t phase_offset)
{
  if (poll_client_len_ >= POLL_CLIENTS_MAX_LEN) return;

  PollClient & client = poll_clients_[poll_client_len_++];
  client.callback = callback;
  client.context = context;
  client.phase_offset = phase_offset;
}

void STM32H7Board::register_exti_client(
  void * context, bool (*matches)(void * context, uint16_t exti_pin), void (*callback)(void * context))
{
  if (exti_client_len_ >= EXTI_CLIENTS_MAX_LEN) return;

  ExtiClient & client = exti_clients_[exti_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Board::register_spi_client(
  void * context, bool (*matches)(void * context, SPI_HandleTypeDef * hspi), void (*callback)(void * context))
{
  if (spi_client_len_ >= SPI_CLIENTS_MAX_LEN) return;

  SpiClient & client = spi_clients_[spi_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Board::register_i2c_client(
  void * context, bool (*matches)(void * context, I2C_HandleTypeDef * hi2c), void (*callback)(void * context))
{
  if (i2c_client_len_ >= I2C_CLIENTS_MAX_LEN) return;

  I2cClient & client = i2c_clients_[i2c_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}


void STM32H7Board::register_adc_client(
  void * context, bool (*matches)(void * context, ADC_HandleTypeDef * hadc),
  void (*callback)(void * context, ADC_HandleTypeDef * hadc))
{
  if (adc_client_len_ >= ADC_CLIENTS_MAX_LEN) return;

  AdcClient & client = adc_clients_[adc_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Board::register_cdc_client(
  void * context, bool (*matches)(void * context, uint8_t chan),
  void (*receive_callback)(void * context, uint8_t * buffer, uint16_t size),
  void (*transmit_cplt_callback)(void * context))
{
  if (cdc_client_len_ >= CDC_CLIENTS_MAX_LEN) return;

  CdcClient & client = cdc_clients_[cdc_client_len_++];
  client.matches = matches;
  client.receive_callback = receive_callback;
  client.transmit_cplt_callback = transmit_cplt_callback;
  client.context = context;
}

void STM32H7Board::register_sd_client(
  void * context, bool (*matches)(void * context, SD_HandleTypeDef * hsd), void (*txcplt_callback)(void * context),
  void (*rxcplt_callback)(void * context))
{
  if (sd_client_len_ >= SD_CLIENTS_MAX_LEN) return;

  SdClient & client = sd_clients_[sd_client_len_++];
  client.matches = matches;
  client.txcplt_callback = txcplt_callback;
  client.rxcplt_callback = rxcplt_callback;
  client.context = context;
}
void STM32H7Board::register_uart_rxcplt_client(
  void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context))
{
  if (uart_rxcplt_client_len_ >= UART_RXCPLT_CLIENTS_MAX_LEN) return;

  UartClient & client = uart_rxcplt_clients_[uart_rxcplt_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Board::register_uart_rxisr_client(
  void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context))
{
  if (uart_rxisr_client_len_ >= UART_RXISR_CLIENTS_MAX_LEN) return;

  UartClient & client = uart_rxisr_clients_[uart_rxisr_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Board::register_uart_txcplt_client(
  void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context))
{
  if (uart_txcplt_client_len_ >= UART_TXCPLT_CLIENTS_MAX_LEN) return;

  UartClient & client = uart_txcplt_clients_[uart_txcplt_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Board::dispatch_poll(uint64_t poll_counter)
{
  for (uint32_t i = 0; i < poll_client_len_; i++) {
    const PollClient & client = poll_clients_[i];
    uint64_t adjusted_poll_counter = poll_counter;

    if (client.phase_offset >= 0) {
      adjusted_poll_counter += static_cast<uint64_t>(client.phase_offset);
    } else {
      const uint64_t phase_offset = static_cast<uint64_t>(-client.phase_offset);
      if (poll_counter < phase_offset) continue;
      adjusted_poll_counter -= phase_offset;
    }

    client.callback(client.context, adjusted_poll_counter);
  }
}

void STM32H7Board::dispatch_exti(uint16_t exti_pin)
{
  for (uint32_t i = 0; i < exti_client_len_; i++) {
    const ExtiClient & client = exti_clients_[i];
    if (!client.matches(client.context, exti_pin)) continue;
    client.callback(client.context);
  }
}

void STM32H7Board::dispatch_spi(SPI_HandleTypeDef * hspi)
{
  for (uint32_t i = 0; i < spi_client_len_; i++) {
    const SpiClient & client = spi_clients_[i];
    if (!client.matches(client.context, hspi)) continue;
    client.callback(client.context);
  }
}

void STM32H7Board::dispatch_i2c(I2C_HandleTypeDef * hi2c)
{
  for (uint32_t i = 0; i < i2c_client_len_; i++) {
    const I2cClient & client = i2c_clients_[i];
    if (!client.matches(client.context, hi2c)) continue;
    client.callback(client.context);
  }
}


void STM32H7Board::dispatch_adc(ADC_HandleTypeDef * hadc)
{
  for (uint32_t i = 0; i < adc_client_len_; i++) {
    const AdcClient & client = adc_clients_[i];
    if (!client.matches(client.context, hadc)) continue;
    client.callback(client.context, hadc);
  }
}

void STM32H7Board::dispatch_cdc_receive(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  if(chan !=0) return;
  for (uint32_t i = 0; i < cdc_client_len_; i++) {
    const CdcClient & client = cdc_clients_[i];
    if (!client.matches(client.context, chan)) continue;
    client.receive_callback(client.context, buffer, size);
  }
}

void STM32H7Board::dispatch_cdc_transmit_cplt(uint8_t chan)
{
  if(chan !=0) return;
  for (uint32_t i = 0; i < cdc_client_len_; i++) {
    const CdcClient & client = cdc_clients_[i];
    if (!client.matches(client.context, chan)) continue;
    client.transmit_cplt_callback(client.context);
  }
}

void STM32H7Board::dispatch_sd_txcplt(SD_HandleTypeDef * hsd)
{
  for (uint32_t i = 0; i < sd_client_len_; i++) {
    const SdClient & client = sd_clients_[i];
    if (!client.matches(client.context, hsd)) continue;
    client.txcplt_callback(client.context);
  }
}

void STM32H7Board::dispatch_sd_rxcplt(SD_HandleTypeDef * hsd)
{
  for (uint32_t i = 0; i < sd_client_len_; i++) {
    const SdClient & client = sd_clients_[i];
    if (!client.matches(client.context, hsd)) continue;
    client.rxcplt_callback(client.context);
  }
}
void STM32H7Board::dispatch_uart_rxcplt(UART_HandleTypeDef * huart)
{
  for (uint32_t i = 0; i < uart_rxcplt_client_len_; i++) {
    const UartClient & client = uart_rxcplt_clients_[i];
    if (!client.matches(client.context, huart)) continue;
    client.callback(client.context);
  }
}

void STM32H7Board::dispatch_uart_rxisr(UART_HandleTypeDef * huart)
{
  for (uint32_t i = 0; i < uart_rxisr_client_len_; i++) {
    const UartClient & client = uart_rxisr_clients_[i];
    if (!client.matches(client.context, huart)) continue;
    client.callback(client.context);
  }
}

void STM32H7Board::dispatch_uart_txcplt(UART_HandleTypeDef * huart)
{
  for (uint32_t i = 0; i < uart_txcplt_client_len_; i++) {
    const UartClient & client = uart_txcplt_clients_[i];
    if (!client.matches(client.context, huart)) continue;
    client.callback(client.context);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////
// us clock
uint32_t STM32H7Board::clock_millis() { return (time64.Us()) / 1000; }
uint64_t STM32H7Board::clock_micros() { return (time64.Us()); }
void STM32H7Board::clock_delay(uint32_t ms) { time64.dMs(ms); }

///////////////////////////////////////////////////////////////////////////////////////////////
// serial comms to the Companion computer
void STM32H7Board::serial_init(uint32_t baud_rate, uint32_t dev)
{
  serial_device_ = USE_TELEM; // dev; // 1 = telem uart, otherwise = VCP
  if (serial_device_ == 1) telem_.reset_baud(baud_rate);
}

void STM32H7Board::serial_write(const uint8_t * src, size_t len, uint8_t qos)
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

uint16_t STM32H7Board::serial_bytes_available(void)
{
  if (serial_device_ == 1) return telem_.byteCount();
  else return vcp_.byteCount();
}

uint8_t STM32H7Board::serial_read(void)
{
  uint8_t c = 0;
  if (serial_device_ == 1) telem_.readByte(&c);
  else vcp_.readByte(&c);
  return c;
}

void STM32H7Board::serial_flush(void)
{
  // do nothing
}

///////////////////////////////////////////////////////////////////////////////////////////////
//
// Sensors
//
//////////////////////////////////////////////////////////////////////////////////////////////////

void STM32H7Board::sensors_init()
{
  sensor_errors_ = 0;
  for (uint32_t i = 0; i < stm32_h7_board.status_len(); i++) {
    if (stm32_h7_board.status(i)->status() != DRIVER_OK) sensor_errors_++;
  }
}
uint16_t STM32H7Board::sensors_errors_count() { return sensor_errors_; }

uint16_t STM32H7Board::sensors_init_message_count() { return stm32_h7_board.status_len(); }

bool STM32H7Board::sensors_init_message_good(uint16_t i)
{ return stm32_h7_board.status(i)->initGood(); }

uint16_t STM32H7Board::sensors_init_message(char * message, uint16_t size, uint16_t i)
{
  if (i > stm32_h7_board.status_len()) return 0;

  uint32_t status = stm32_h7_board.status(i)->status();
  if (status == DRIVER_OK) {
    snprintf(message, size, "%s: INIT OK", stm32_h7_board.status(i)->name());
  } else { //PTT TODO: we can add better messages later
    snprintf(message, size, "%s: INIT ERROR 0x%08lX", stm32_h7_board.status(i)->name(), status);
  }
  return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// IMU
bool STM32H7Board::imu_read(rosflight_firmware::ImuStruct * imu)
{
  ImuPacket p;
  if (imu0_.read((uint8_t *) &p, sizeof(p))) {
    imu->header = p.header;
    imu->accel[0] = p.accel[0];
    imu->accel[1] = p.accel[1];
    imu->accel[2] = p.accel[2];
    imu->gyro[0] = p.gyro[0];
    imu->gyro[1] = p.gyro[1];
    imu->gyro[2] = p.gyro[2];
    imu->temperature = p.temperature;
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// MAG
bool STM32H7Board::mag_read(rosflight_firmware::MagStruct * mag)
{
  MagPacket p;
  if (mag_.read((uint8_t *) &p, sizeof(p))) {
    mag->header = p.header;
    mag->flux[0] = p.flux[0];
    mag->flux[1] = p.flux[1];
    mag->flux[2] = p.flux[2];
    mag->temperature = p.temperature;
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Baro
bool STM32H7Board::baro_read(rosflight_firmware::PressureStruct * baro)
{
  PressurePacket p;
  if (baro_.read((uint8_t *) &p, sizeof(p))) {
    baro->header = p.header;
    baro->pressure = p.pressure;
    baro->temperature = p.temperature;
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Pitot
bool STM32H7Board::diff_pressure_read(rosflight_firmware::PressureStruct * diff_pressure)
{
  PressurePacket p;
  if (pitot_.read((uint8_t *) &p, sizeof(p))) {
    diff_pressure->header = p.header;
    diff_pressure->pressure = p.pressure;
    diff_pressure->temperature = p.temperature;
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Sonar
bool STM32H7Board::range_read(rosflight_firmware::RangeStruct * range)
{
  (void) range; // unused
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Battery
bool STM32H7Board::battery_read(rosflight_firmware::BatteryStruct * batt)
{
  AdcPacket p;
  if (adc_.read((uint8_t *) &p, sizeof(p))) {
    batt->header = p.header;
    batt->current = p.battery_current;
    batt->voltage = p.battery_voltage;
    batt->temperature = p.temperature;
    return true;
  }
  return false;
}
void STM32H7Board::battery_voltage_set_multiplier(double multiplier)
{
  if (multiplier == 0) return;
  adc_.setScaleFactor(adc_.battery_voltage_index(), multiplier);
}
void STM32H7Board::battery_current_set_multiplier(double multiplier)
{
  if (multiplier == 0) return;
  adc_.setScaleFactor(adc_.battery_current_index(), multiplier);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// GNSS
bool STM32H7Board::gnss_read(rosflight_firmware::GnssStruct * gnss)
{
  UbxPacket p;

  if (gps_.read((uint8_t *) &p, sizeof(p))) {
    gnss->header = p.header;
    gnss->pps = p.pps;
    gnss->unix_seconds = p.unix_seconds; // Unix time
    gnss->unix_nanos = p.unix_nanos;
    gnss->fix_type = p.pvt.fixType;
    gnss->num_sat = p.pvt.numSV;
    gnss->lon = (double) p.pvt.lon * 1e-7;        // Convert 100's of nanodegs into deg (DDS format)
    gnss->lat = (double) p.pvt.lat * 1e-7;        // Convert 100's of nanodegs into deg (DDS format)
    gnss->height_msl = (float) p.pvt.hMSL * 1e-3; //mm to m
    gnss->vel_n = (float) p.pvt.velN * 1e-3;      // mm/s to m/s
    gnss->vel_e = (float) p.pvt.velE * 1e-3;      // mm/s to m/s
    gnss->vel_d = (float) p.pvt.velD * 1e-3;      // mm/s to m/s
    gnss->h_acc = (float) p.pvt.hAcc * 1e-3;      //mm to m
    gnss->v_acc = (float) p.pvt.vAcc * 1e-3;      //mm to m
    gnss->speed_accy = (float) p.pvt.sAcc * 1e-3; // mm/s to m/s
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// RC
void STM32H7Board::rc_init(rc_type_t rc_type) { (void) rc_type; };
bool STM32H7Board::rc_read(rosflight_firmware::RcStruct * rc_struct)
{
  RcPacket p;

  if (rc_.read((uint8_t *) &p, sizeof(p))) {
    rc_struct->header = p.header;
    uint16_t len =
      RC_STRUCT_CHANNELS < RC_PACKET_CHANNELS ? RC_STRUCT_CHANNELS : RC_PACKET_CHANNELS;
    for (uint16_t i = 0; i < len; i++) { rc_struct->chan[i] = p.chan[i]; }
    rc_struct->frameLost = p.frameLost;
    rc_struct->failsafeActivated = p.failsafeActivated;
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// PWM

void STM32H7Board::pwm_init(const float * rate, uint32_t channels)
{ pwm_.updateConfig(rate, channels); }
void STM32H7Board::pwm_disable(void)
{
  for (uint32_t ch = 0; ch < pwm_.channel_count(); ch++) pwm_.disable(ch);
}
void STM32H7Board::pwm_write(float * value, uint32_t channels) { pwm_.write(value, channels); }

///////////////////////////////////////////////////////////////////////////////////////////////
// LEDs
void STM32H7Board::led0_on() { RED_HI; }
void STM32H7Board::led0_off() { RED_LO; }
void STM32H7Board::led0_toggle() { RED_TOG; }

void STM32H7Board::led1_on() { BLU_HI; }
void STM32H7Board::led1_off() { BLU_LO; }
void STM32H7Board::led1_toggle() { BLU_TOG; }

///////////////////////////////////////////////////////////////////////////////////////////////
// Backup Data (Register and SRAM)
// https://stackoverflow.com/questions/20667754/how-to-use-backup-sram-as-eeprom-in-stm32f4
// from \Drivers\CMSIS\Device\ST\STM32H7xx\Include\stm32h753xx.h(2141)
//#define D3_BKPSRAM_BASE           (0x38800000UL) /*!< Base address of : Backup SRAM(4 KB) over AXI->AHB Bridge */
//#define D3_SRAM_BASE              (0x38000000UL) /*!< Base address of : Backup SRAM(64 KB) over AXI->AHB Bridge

#define D3_BKPSRAM_BASE_LEN (4096U)
#define D3_SRAM_BASE_LEN (65536U)
void STM32H7Board::backup_memory_init() {}
bool STM32H7Board::backup_memory_read(void * dest, size_t len)
{
  //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
  //	HAL_PWR_EnableBkUpAccess();
  //	memcpy(dest, (void*)D3_BKPSRAM_BASE, len);
  //	HAL_PWR_DisableBkUpAccess();
  return true;
}
void STM32H7Board::backup_memory_write(const void * src, size_t len)
{
  //	if(len > D3_BKPSRAM_BASE_LEN) len = D3_BKPSRAM_BASE_LEN;
  //	HAL_PWR_EnableBkUpAccess();
  //	memcpy((void*)D3_BKPSRAM_BASE, src, len);
  //	HAL_PWR_DisableBkUpAccess();
}
void STM32H7Board::backup_memory_clear(size_t len)
{
  //	HAL_PWR_EnableBkUpAccess();
  //	memset((void*)D3_BKPSRAM_BASE, 0, D3_BKPSRAM_BASE_LEN);
  //	HAL_PWR_DisableBkUpAccess();
}

void STM32H7Board::memory_init() {} // do nothing

bool STM32H7Board::memory_read(void * dest, size_t len) { return sd_.read((uint8_t *) dest, len); }
bool STM32H7Board::memory_write(const void * src, size_t len)
{ return sd_.write((uint8_t *) src, len); }

///////////////////////////////////////////////////////////////////////////////////////////////
// Helper functions (not part of parent class)

