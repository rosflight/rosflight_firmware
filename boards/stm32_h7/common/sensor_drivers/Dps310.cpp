/**
 ******************************************************************************
 * File     : Dps310.cpp
 * Date     : Sep 28, 2023
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

#include "Dps310.h"

#include "Packets.h"
#include "Time64.h"
#include "misc.h"
#include "stm32_h7.hpp"

#include <cstring>

#define DPS310_CONTINUOUS_MODE false

#define SPI_WRITE ((uint8_t) 0x00)
#define SPI_READ ((uint8_t) 0x80)

#define DPS310_READ_T_CMD (0x00 | SPI_READ)
#define DPS310_READ_T_BUFFBYTES (12) // read this may to clear drdy registers

#define DPS310_READ_P_CMD (0x00 | SPI_READ)
#define DPS310_READ_P_BUFFBYTES (12) // read this may to clear drdy registers

#define KP 7864320.0 // 8x oversample
#define KT 524288.0  // No oversampling
//	#define KP1  524288.0 // 2 times
//	#define KP2 1572864.0 // 2 times
//	#define KP4 3670016.0 // 4 times
//	#define KP8 7864320.0 // 8 times
//   etc.

extern Time64 time64;

DTCM_RAM uint8_t dps310_double_buffer[2 * sizeof(PressurePacket)];

#define ROLLOVER 20000
#define DPS310_CMD_P 0
#define DPS310_DRDY_P 145
#define DPS310_RX_P 146
#define DPS310_CMD_T 147
#define DPS310_DRDY_T 177
#define DPS310_RX_T 178

static int32_t Compliment(int32_t x, int16_t bits)
{
  if (x & ((int32_t) 1 << (bits - 1))) { x -= (int32_t) 1 << bits; }
  return x;
}

//PTT note if there is an issue when setting 3-wire mode, need to add
//int16_t DpsClass::correctTemp(void)
//{
//	writeByte(0x0E, 0xA5);
//	writeByte(0x0F, 0x96);
//	writeByte(0x62, 0x02);
//	writeByte(0x0E, 0x00);
//	writeByte(0x0F, 0x00);
//
//	//perform a first temperature measurement (again)
//	//the most recent temperature will be saved internally
//	//and used for compensation when calculating pressure
//	float trash;
//	measureTempOnce(trash);
//
//	return DPS__SUCCEEDED;
//}

uint32_t Dps310::init(
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, uint16_t drdy_pin, SPI_HandleTypeDef * hspi,
  GPIO_TypeDef * cs_port, uint16_t cs_pin, bool three_wire)
{
  (void) drdy_port;
  (void) drdy_pin;
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Dps310");
  initializationStatus_ = DRIVER_OK;

  sampleRateHz_ = sample_rate_hz;
  drdy_ = 0;

  uint8_t init_txbuf[2] = {};
  uint8_t init_rxbuf[2] = {};
  Spi init_spi;
  init_spi.init(hspi, init_txbuf, init_rxbuf, cs_port, cs_pin);
  async_device_.cs_port = cs_port;
  async_device_.cs_pin = cs_pin;

  timeoutMs_ = 100;
  // groupDelay_		= 1000000/sampleRateHz_;
  HAL_GPIO_WritePin(init_spi.port_, init_spi.pin_, GPIO_PIN_SET);

  double_buffer_.init(dps310_double_buffer, sizeof(dps310_double_buffer));

#define RESET 0x0C
  const auto write_register = [&](uint8_t address, uint8_t value) {
    uint8_t tx[2] = {(uint8_t) (address | SPI_WRITE), value};
    init_spi.tx(tx, 2, timeoutMs_);
  };

  const auto read_register = [&](uint8_t address) {
    uint8_t tx[2] = {(uint8_t) (address | SPI_READ), 0};
    uint8_t rx[2] = {0};
    init_spi.rx(tx, rx, 2, timeoutMs_);
    return rx[1];
  };

  write_register(RESET, 0x09);
  HAL_Delay(40);

  // Set to 3-wire SPI mode so we can read registers.
  // Interrupt and FIFO Config 0x09
  // 7 - 	1, DRDY active high
  // 6 - 	0, Disable FIFO full interrupt
  // 5 - 	0, Int on temp
  // 4 - 	1, Int on pressure
  // 3 - 	0, no Temp data shift
  // 2 - 	0, no Press data shift
  // 1 - 	0, Disable FIFO
  // 0 - 	1, 3-wire SPI interface
#define CFG_REG 0x09
  if (three_wire) write_register(CFG_REG, 0x01);
  else write_register(CFG_REG, 0x00);

    // Product ID 0x0D
#define PRODUCT_ID 0x0D
  uint8_t product_id = read_register(PRODUCT_ID);
  misc_printf("DPS310: PRODUCT ID = 0x%02X  (0x10) -", product_id);
  if (product_id == 0x10) misc_printf(" OK\n");
  else {
    initializationStatus_ |= DRIVER_ID_MISMATCH;
    misc_printf(" Not OK\n");
  }

  // Calibration constants
#define MEAS_CFG 0x08
  uint8_t coef_rdy = read_register(MEAS_CFG) & 0x80;

  for (int n = 0; n < 10; n++) // Wait 10 times for Coefficients to be ready
  {
    coef_rdy = read_register(MEAS_CFG) & 0x80;
    //		misc_printf("DPS310: COEF_RDY   = 0x%02X\n",coef_rdy);
    if ((coef_rdy & 0x80) == 0x80) break;
    time64.dUs(1000);
  }
  misc_printf("DPS310: COEF_RDY = 0x%02X (0x80) ", coef_rdy);
  if ((coef_rdy & 0x80) == 0x80) misc_printf("- READY\n");
  else {
    misc_printf("- NOT READYn\n");
    initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;
  }

  misc_printf("DPS310: Reading Coefficients\n");

  // Read Calibration Constants
#define COEF_REG 0x10
  uint8_t tx[19] = {COEF_REG | SPI_READ, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t rx[19];

  init_spi.rx(tx, rx, 19, timeoutMs_);

  int32_t buf[18];
  for (int n = 0; n < 18; n++) buf[n] = rx[n + 1];
  int32_t C0, C1, C01, C11, C20, C21, C30; // Calibration Constants
  int32_t C00, C10;

  C0 = (buf[0] << 4) | ((buf[1] >> 4) & 0x0F);
  C1 = ((buf[1] & 0x0F) << 8) | buf[2];
  C00 = (buf[3] << 12) | (buf[4] << 4) | ((buf[5] >> 4) & 0x0F);
  C10 = ((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7];

  C01 = (buf[8] << 8) | buf[9];
  C11 = (buf[10] << 8) | buf[11];
  C20 = (buf[12] << 8) | buf[13];
  C21 = (buf[14] << 8) | buf[15];
  C30 = (buf[16] << 8) | buf[17];

  C0_ = Compliment(C0, 12);
  C1_ = Compliment(C1, 12);
  C00_ = Compliment(C00, 20);
  C10_ = Compliment(C10, 20);
  C01_ = Compliment(C01, 16);
  C11_ = Compliment(C11, 16);
  C20_ = Compliment(C20, 16);
  C21_ = Compliment(C21, 16);
  C30_ = Compliment(C30, 16);

  misc_printf("DPS310: C0,  C1  = %10.0f %10.0f\n", C0_, C1_);
  misc_printf("DPS310: C00, C10 = %10.0f %10.0f\n", C00_, C10_);
  misc_printf("DPS310: C01, C11 = %10.0f %10.0f\n", C01_, C11_);
  misc_printf("DPS310: C20, C21 = %10.0f %10.0f\n", C20_, C21_);
  misc_printf("DPS310: C30      = %10.0f\n", C30_);

#define COEF_SRCE 0x28
  uint8_t temp_source = read_register(COEF_SRCE) & 0x80;
  misc_printf("DPS310: temp source = 0x%02X\n", temp_source);

#define PRS_CFG 0x06            // Pressure Configuration
  write_register(PRS_CFG, 0x63); // 64 measurements per second, 8x oversampling

#define TMP_CFG 0x07                          // Temperature Configuration
  write_register(TMP_CFG, temp_source | 0x60); // 64 measurements per second, no oversampling

  // Interrupt and FIFO Config 0x09
  // 7 - 	1, DRDY active high
  // 6 - 	0, Disable FIFO full interrupt
  // 5 - 	0, Int on temp
  // 4 - 	1, Int on pressure
  // 3 - 	0, no Temp data shift
  // 2 - 	0, no Press data shift
  // 1 - 	0, Disable FIFO
  // 0 - 	1, 3-wire SPI interface
  // 1001 0001 = 0x91
  // 1011 0001 = 0xB1
  if (three_wire) {
#if DPS310_CONTINUOUS_MODE
    write_register(CFG_REG,
                   0x91); // Interrupt on T only 3-wire supports interrupts, 4-wire does not support interrupts
#else
    write_register(CFG_REG, 0xB1); // Interrupt on both P and T
#endif
  } else {
#if DPS310_CONTINUOUS_MODE
    write_register(CFG_REG,
                   0x90); // Interrupt on T only 3-wire supports interrupts, 4-wire does not support interrupts
#else
    write_register(CFG_REG, 0xB0); // Interrupt on both P and T
#endif
  }

  // Measurement Configuration
  // 7 - 	0, read only
  // 6 - 	0, read only
  // 5 - 	0, read only
  // 4 - 	0, read only
  // 3 - 	0, reserved
  // 2:0 - 	111, pressure and temperature continuous mode
  // 0000 0111 =  0x07
#if DPS310_CONTINUOUS_MODE
  write_register(MEAS_CFG, 0x07); // Start background measurement
#endif

  //PTT need to add
  //	writeByte(0x0E, 0xA5);
  //	writeByte(0x0F, 0x96);
  //	writeByte(0x62, 0x02);
  //	writeByte(0x0E, 0x00);
  //	writeByte(0x0F, 0x00);
  //
  //	//perform a first temperature measurement (again)
  //	//the most recent temperature will be saved internally
  //	//and used for compensation when calculating pressure
  //	float trash;
  //	measureTempOnce(trash);

  return initializationStatus_;
}

bool Dps310::poll(uint64_t poll_counter)
{
  uint16_t poll_state;
  if (!stm32_h7_board.polling_timer().polling_state(poll_counter, ROLLOVER, poll_state)) return false;
  if (async_bus_ == nullptr) return false;

  poll_signal_.tick(poll_counter);
  if (poll_state == DPS310_CMD_P) {
    drdy_ = time64.Us();
    poll_signal_.trigger();
  }
  return false;
}

AsyncTask<void> Dps310::run()
{
  PressurePacket p = {};
  double Traw = 0.0;
  uint8_t tx[DPS310_READ_T_BUFFBYTES] = {};
  uint8_t rx[DPS310_READ_T_BUFFBYTES] = {};

  while (true) {
    co_await poll_signal_.wait_for_trigger();
    tx[0] = MEAS_CFG | SPI_WRITE;
    tx[1] = 0x01;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, 2)).status != AsyncStatus::OK) {
      continue;
    }

    co_await poll_signal_.delay_ticks(DPS310_DRDY_P - DPS310_CMD_P);
    tx[0] = MEAS_CFG | SPI_READ;
    tx[1] = 0;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, 2)).status != AsyncStatus::OK) {
      continue;
    }
    if (rx[1] & 0x10) {
      p.header.status |= (uint16_t) rx[1];
      p.header.complete = time64.Us();
    }

    co_await poll_signal_.delay_ticks(DPS310_RX_P - DPS310_DRDY_P);
    std::memset(tx, 0, DPS310_READ_P_BUFFBYTES);
    tx[0] = DPS310_READ_P_CMD;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, DPS310_READ_P_BUFFBYTES)).status != AsyncStatus::OK) {
      continue;
    }
    int32_t praw = ((int32_t) rx[1] << 24 | (int32_t) rx[2] << 16 | (int32_t) rx[3] << 8) >> 8;
    double Praw = (double) praw / KP;
    p.pressure = C00_ + Praw * (C10_ + Praw * (C20_ + Praw * C30_)) + Traw * (C01_ + Praw * (C11_ + Praw * C21_));
    p.header.timestamp = drdy_;
    p.header.complete = time64.Us();
    if (p.header.status == DPS310_OK) write((uint8_t *) &p, sizeof(p));
    p.header.status = 0;

    co_await poll_signal_.delay_ticks(DPS310_CMD_T - DPS310_RX_P);
    tx[0] = MEAS_CFG | SPI_WRITE;
    tx[1] = 0x02;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, 2)).status != AsyncStatus::OK) {
      continue;
    }

    co_await poll_signal_.delay_ticks(DPS310_DRDY_T - DPS310_CMD_T);
    tx[0] = MEAS_CFG | SPI_READ;
    tx[1] = 0;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, 2)).status != AsyncStatus::OK) {
      continue;
    }
    if (rx[1] & 0x20) p.header.status = (uint16_t) rx[1] << 8;

    co_await poll_signal_.delay_ticks(DPS310_RX_T - DPS310_DRDY_T);
    std::memset(tx, 0, DPS310_READ_T_BUFFBYTES);
    tx[0] = DPS310_READ_T_CMD;
    if ((co_await async_bus_->transfer(async_device_, tx, rx, DPS310_READ_T_BUFFBYTES)).status != AsyncStatus::OK) {
      continue;
    }
    int32_t traw = ((int32_t) rx[4] << 24 | (int32_t) rx[5] << 16 | (int32_t) rx[6] << 8) >> 8;
    Traw = (double) traw / KT;
    p.temperature = C0_ * 0.5 + C1_ * Traw + 273.15;
  }
}

bool Dps310::display(void)
{
  PressurePacket p;

  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name_, p.header);
    misc_f32(98, 101, p.pressure / 1000., "Press", "%6.2f", "kPa");
    misc_f32(18, 50, p.temperature - 273.15, "Temp", "%5.1f", "C");
    misc_x16(DPS310_OK, p.header.status, "Status");
    misc_printf("\n");
    return 1;
  } else {
    misc_printf("%s\n", name_);
  }
  return 0;
}

void Dps310::register_callbacks(STM32H7Board & board, int32_t poll_phase_offset)
{
  if (async_bus_ == nullptr) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return;
  }

  board.callbacks().register_poll_client(this, poll_phase_offset);
  task_ = run();
}


