/**
 ******************************************************************************
 * File     : Iis2mdc.cpp
 * Date     : Sep 29, 2023
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

#include "Iis2mdc.h"

#include "Packets.h"
#include "Spi.h"
#include "Time64.h"
#include "misc.h"
#include "stm32_h7.hpp"

#include <cstring>

extern Time64 time64;

#define WHO_AM_I 0x4F
#define CFG_REG_A 0x60
#define CFG_REG_B 0x61
#define CFG_REG_C 0x62
#define OFFSET_REG 0x45
#define STATUS_REG 0x67
#define OUT_TEMP 0x6E

#define SPI_WRITE 0x00
#define SPI_READ 0x80

#define IIS_FLUX_CMD (STATUS_REG | SPI_READ) // read both status and flux
#define IIS_FLUX_BYTES 8
#define IIS_TEMP_CMD (OUT_TEMP | SPI_READ)
#define IIS_TEMP_BYTES 3

DTCM_RAM uint8_t iis2mdc_double_buffer[2 * sizeof(MagPacket)];

#define ROLLOVER 10000
#define IIS2MDC_CMD 0
#define IIS2MDC_RX_H 97
#define IIS2MDC_RX_T (IIS2MDC_RX_H + 1)

uint32_t Iis2mdc::init(
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, uint16_t drdy_pin, SPI_HandleTypeDef * hspi,
  GPIO_TypeDef * cs_port, uint16_t cs_pin, const double *rotation)
{
  (void) drdy_port;
  (void) drdy_pin;
  memcpy(rotation_, rotation, sizeof(double) * 9);
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Iis2mdc");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  drdy_ = 0;
  Spi init_spi;
  init_spi.init(hspi, cs_port, cs_pin);
  async_device_.cs_port = cs_port;
  async_device_.cs_pin = cs_pin;

  HAL_GPIO_WritePin(init_spi.port_, init_spi.pin_, GPIO_PIN_SET);

  double_buffer_.init(iis2mdc_double_buffer, sizeof(iis2mdc_double_buffer));


  const auto write_register = [&](uint8_t address, uint8_t value) {
    uint8_t tx[2] = {(uint8_t) (address | SPI_WRITE), value};
    init_spi.tx(tx, 2, 100);
  };

  const auto read_register = [&](uint8_t address) {
    uint8_t tx[2] = {(uint8_t) (address | SPI_READ), 0};
    uint8_t rx[2] = {0};
    HAL_StatusTypeDef hal_status = init_spi.rx(tx, rx, 2, 100);
    return (uint8_t) (rx[1] | hal_status);
  };

  //	uint8_t odr_mode=3;
  //	if( sampleRateHz_ <= 10) 		{sampleRateHz_ =  10; odr_mode = 0; }
  //	else if( sampleRateHz_ <= 20) 	{sampleRateHz_ =  20; odr_mode = 1; }
  //	else if( sampleRateHz_ <= 50) 	{sampleRateHz_ =  50; odr_mode = 2; }
  //	else if( sampleRateHz_ <= 100) 	{sampleRateHz_ = 100; odr_mode = 3; }
  //	else 			                {sampleRateHz_ = 100; odr_mode = 3; }

  uint8_t id = read_register(WHO_AM_I);

  misc_printf("Iis2mdc: WHO_AM_I = 0x%02X (0x40) - ", id);
  if (id == 0x40) misc_printf(" Matches\n");
  else {
    misc_printf(" Does not match\n");
    initializationStatus_ |= DRIVER_ID_MISMATCH;
  }

  // Reboot the sensor
  // Register A (0x60)
  // 7:  = 0 COMP_TEMP_EN Temp comp enable
  // 6:  = X REBOOT
  // 5:  = X SOFT_RST
  // 4:  = 0 High resolution Mode (LP=0)
  // 3:2 = 00 10 Hz Data Rate (ODR)
  // 1:0 = 00 Continuous Mode
  write_register(CFG_REG_A, 0x20); // soft reset
  time64.dUs(10);                 // Wait at least 5 us
  write_register(CFG_REG_A, 0x40); // reboot
  time64.dMs(25);                 // wait at least 20 ms for reboot

  // Register A (0x60)
  // 7:  = 1 COMP_TEMP_EN Temp comp enable
  // 6:  = 0 REBOOT
  // 5:  = 0 SOFT_RST
  // 4:  = 0 High resolution Mode (High resolution = 0, Low power =1)
  //
  // 3:2 = 11 = 100 Hz Data Rate (ODR)
  // 1:0 = 00 Continuous Mode, 01 = single mode
  //	write_register(0x60,0x81); // 1000 0001 = 0x81 For Single Acq
  // 1000 1100 = 0x8C For 100 Hz.
  // write_register(CFG_REG_A,0x80|(odr_mode<<2)); // continuous mode
  write_register(CFG_REG_A, 0x83); // Set to idle
  // Register B (0x61)
  // [7:5] 000
  // [4]  1 OFF_CANC_ONE_SHOT 1=Offset Cancellation in single mode
  // [3]  0
  // [2]  0 Set Freq of Set pulse to 63 ODR
  // [1]  1, OFF_CANC 1= enable offset cancellation in single mode
  // [0]  0 LPF disable offset filter (1- enabled)
  write_register(CFG_REG_B, 0x12); // 0001 0010 For Single Mode (alternating set/reset)
  // Register C (0x62)
  // 7: =0 Unused
  // 6: =0 INT_on_PIN Enable event interrupts
  // 5: =1 I2C_DIS (Disable I2C interface use only SPI)
  // 4: =1 BDU
  //
  // 3: =0 BLE do not swap data bytes
  // 2: =0 Unused
  // 1: =0 SELF_TEST
  // 0: =1 DRDY_on_PIN Enable DRDY
  write_register(CFG_REG_C, 0x31); // 0011 0001 = 0x31 // 0011 1001 = 0x39

  // INT_CTRL_REG (0x63)
  // Disable Interrupts (this is not DRDY)
  write_register(0x63, 0x00);
  write_register(0x64, 0x00);
  write_register(0x65, 0x00);
  write_register(0x66, 0x00);

  // Read Status Register (0x67)
  uint8_t sensor_status = read_register(STATUS_REG);
  misc_printf("IIS2MDC: Mag status register = 0x%02X (0x00)\n", sensor_status);
  if (sensor_status != 0x00) initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;

  // Read Offset Registers (6 bytes starting 0x45)
  uint8_t tx[7], h[7];
  memset(tx, 0, sizeof(tx));
  tx[0] = OFFSET_REG | SPI_READ;
  init_spi.rx(tx, h, 7, 100);
  misc_printf("H Offsets should be zero %8d %8d %8d mGauss\n", ((int16_t) h[1] | (int16_t) h[2] << 8) * 3 / 2,
              ((int16_t) h[3] | (int16_t) h[4] << 8) * 3 / 2, ((int16_t) h[5] | (int16_t) h[6] << 8) * 3 / 2);

  return initializationStatus_;
}

bool Iis2mdc::poll(uint64_t poll_counter)
{
  uint16_t poll_state;
  if (!stm32_h7_board.polling_timer().polling_state(poll_counter, ROLLOVER, poll_state)) return false;
  if (async_bus_ == nullptr) return false;

  poll_signal_.tick(poll_counter);
  if (poll_state == IIS2MDC_CMD) {
    drdy_ = time64.Us();
    poll_signal_.trigger();
  }
  return false;
}

AsyncTask<void> Iis2mdc::run()
{
  MagPacket p = {};
  int16_t previous_data[3] = {0, 0, 0};
  uint8_t tx[IIS_FLUX_BYTES] = {};
  uint8_t rx[IIS_FLUX_BYTES] = {};

  while (true) {
    co_await poll_signal_.wait_for_trigger();
    tx[0] = CFG_REG_A | SPI_WRITE;
    tx[1] = 0x81;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, 2)).status != AsyncStatus::OK) {
      continue;
    }

    co_await poll_signal_.delay_ticks(IIS2MDC_RX_H - IIS2MDC_CMD);
    std::memset(tx, 0, IIS_FLUX_BYTES);
    tx[0] = IIS_FLUX_CMD;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, IIS_FLUX_BYTES)).status != AsyncStatus::OK) {
      continue;
    }

    std::memset(&p, 0, sizeof(p));
    p.header.complete = time64.Us();
    p.header.status = rx[1];

    int16_t data = (rx[3] << 8) | rx[2];
    p.flux[0] = (data + previous_data[0]) / 2. * 1.5e-7;
    previous_data[0] = data;

    data = (rx[5] << 8) | rx[4];
    p.flux[1] = (data + previous_data[1]) / 2. * 1.5e-7;
    previous_data[1] = data;

    data = -((rx[7] << 8) | rx[6]);
    p.flux[2] = (data + previous_data[2]) / 2. * 1.5e-7;
    previous_data[2] = data;

    co_await poll_signal_.delay_ticks(IIS2MDC_RX_T - IIS2MDC_RX_H);
    std::memset(tx, 0, IIS_TEMP_BYTES);
    tx[0] = IIS_TEMP_CMD;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, IIS_TEMP_BYTES)).status != AsyncStatus::OK) {
      continue;
    }

    data = (rx[2] << 8) | rx[1];
    p.temperature = (double) data / 8.0 + 25.0 + 273.15;

    p.header.timestamp = drdy_;
    p.header.complete = time64.Us();
    if (p.header.status == IIS2MDC_OK) {
      rotate(p.flux);
      write((uint8_t *) &p, sizeof(p));
    }
  }
}

bool Iis2mdc::display()
{
  MagPacket p;
  char name[] = "Iis2mdc (mag)";
  if (read((uint8_t *) &p, sizeof(p))) {
    float total_flux = sqrt(p.flux[0] * p.flux[0] + p.flux[1] * p.flux[1] + p.flux[2] * p.flux[2]);

    misc_header(name, p.header);
    misc_f32(NAN, NAN, p.flux[0] * 1e6, "hx", "%6.2f", "uT");
    misc_f32(NAN, NAN, p.flux[1] * 1e6, "hy", "%6.2f", "uT");
    misc_f32(NAN, NAN, p.flux[2] * 1e6, "hz", "%6.2f", "uT");
    misc_f32(20, 100, total_flux * 1e6, "|h|", "%6.2f", "uT");
    misc_f32(18, 50, p.temperature - 273.15, "Temp", "%5.1f", "C");
    misc_x16(IIS2MDC_OK, p.header.status, "Status");
    misc_printf("\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }

  return 0;
}

void Iis2mdc::start(STM32H7Board & board, int32_t poll_phase_offset)
{
  if (async_bus_ == nullptr) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return;
  }

  board.callbacks().register_poll_client(this, poll_phase_offset);
  task_ = run();
}



