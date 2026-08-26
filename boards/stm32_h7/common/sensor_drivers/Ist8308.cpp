/**
 ******************************************************************************
 * File     : IST8308.cpp
 * Date     : Jun 18, 2024
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

#include "Ist8308.h"

#include "Time64.h"
#include "misc.h"
#include "stm32_h7.hpp"

extern Time64 time64;

#define ROLLOVER 10000
#define IST8308_CMD 0
#define IST8308_TX 89
#define IST8308_RX 92
DTCM_RAM uint8_t ist8308_double_buffer[2 * sizeof(MagPacket)];

#define WAI_REG 0x0
#define DEVICE_ID 0x08

#define STAT1_REG 0x10
#define STAT1_VAL_DRDY 0x1

#define CNTL2_REG 0x31
#define CNTL2_VAL_SINGLE_MODE 0x1

#define CNTL3_REG 0x32
#define CNTL3_VAL_SRST 1

#define CNTL4_REG 0x34
#define CNTL4_VAL_DYNAMIC_RANGE_500 0

#define OSRCNTL_REG 0x41
#define OSRCNTL_VAL_XZ_16 (4)
#define OSRCNTL_VAL_Y_16 (4 << 3)

uint32_t Ist8308::init(uint16_t sample_rate_hz, I2C_HandleTypeDef * hi2c, uint16_t i2c_address, const double *rotation)
{
  memcpy(rotation_, rotation, sizeof(double) * 9);
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Ist8308");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  address_ = i2c_address << 1;


  double_buffer_.init(ist8308_double_buffer, sizeof(ist8308_double_buffer));

  drdy_ = 0;

  // Check if we are the right chip
  uint8_t reg = WAI_REG;
  if (HAL_I2C_Master_Transmit(hi2c, address_, &reg, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  uint8_t device_id = 0;
  if (HAL_I2C_Master_Receive(hi2c, address_, &device_id, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  misc_printf("IST8308 Device ID = 0x%02X (0x%02X) - ", device_id, DEVICE_ID);
  if (device_id == DEVICE_ID) {
    misc_printf("OK\n");
  } else {
    misc_printf("ERROR\n");
    initializationStatus_ |= DRIVER_ID_MISMATCH;
    return initializationStatus_;
  }

  uint8_t cmd[2];
  // Reset
  cmd[0] = CNTL3_REG;
  cmd[1] = CNTL3_VAL_SRST;
  if (HAL_I2C_Master_Transmit(hi2c, address_, cmd, 2, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  HAL_Delay(20);

  // Write CNTL3_REG (to check status?)
  reg = CNTL3_REG;
  if (HAL_I2C_Master_Transmit(hi2c, address_, &reg, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  uint8_t cntl3 = 0;
  if (HAL_I2C_Master_Receive(hi2c, address_, &cntl3, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  misc_printf("IST8308 Device CNTL3 = 0x%02X, bit 1 = 0x%02X\n", cntl3, cntl3 & 0x01);

  if ((cntl3 & 0x01) != 0) {
    initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;
    return initializationStatus_;
  }

  // Configure
  //	cmd[0] = CNTL3_REG; cmd[1] = CNTL3_VAL_DRDY_EN;
  //	if(HAL_I2C_Master_Transmit (hi2c_, address_, cmd, 2, 1000)!=HAL_OK) return DRIVER_HAL_ERROR;

  cmd[0] = CNTL4_REG;
  cmd[1] = CNTL4_VAL_DYNAMIC_RANGE_500;
  if (HAL_I2C_Master_Transmit(hi2c, address_, cmd, 2, 1000) != HAL_OK) return DRIVER_HAL_ERROR;

  cmd[0] = OSRCNTL_REG;
  cmd[1] = OSRCNTL_VAL_Y_16 | OSRCNTL_VAL_XZ_16;
  if (HAL_I2C_Master_Transmit(hi2c, address_, cmd, 2, 1000) != HAL_OK) return DRIVER_HAL_ERROR;

  cmd[0] = CNTL2_REG;
  cmd[1] = CNTL2_VAL_SINGLE_MODE;
  if (HAL_I2C_Master_Transmit(hi2c, address_, cmd, 2, 1000) != HAL_OK) return DRIVER_HAL_ERROR;

  return initializationStatus_;
}

bool Ist8308::poll(uint64_t poll_counter)
{
  uint16_t poll_state;
  if (!stm32_h7_board.polling_timer().polling_state(poll_counter, ROLLOVER, poll_state)) return false;
  if (async_bus_ == nullptr) return false;

  poll_signal_.tick(poll_counter);
  if (poll_state == IST8308_CMD) {
    poll_signal_.trigger();
  }
  return false;
}

AsyncTask<void> Ist8308::run()
{
  uint8_t tx[2] = {};
  uint8_t rx[7] = {};

  while (true) {
    co_await poll_signal_.wait_for_trigger();
    tx[0] = CNTL2_REG;
    tx[1] = CNTL2_VAL_SINGLE_MODE;
    if ((co_await async_bus_->write(address_, tx, 2)).status != AsyncStatus::OK) {
      continue;
    }

    co_await poll_signal_.delay_ticks(IST8308_TX - IST8308_CMD);
    drdy_ = time64.Us();
    tx[0] = STAT1_REG;
    if ((co_await async_bus_->write(address_, tx, 1)).status != AsyncStatus::OK) {
      continue;
    }

    co_await poll_signal_.delay_ticks(IST8308_RX - IST8308_TX);
    if ((co_await async_bus_->read(address_, rx, 7)).status != AsyncStatus::OK) {
      continue;
    }

    MagPacket p;
    p.header.timestamp = drdy_;
    p.header.complete = time64.Us();
    p.header.status = rx[0];

    if (p.header.status == STAT1_VAL_DRDY) {
      p.temperature = 0;

      int16_t iflux = ((int16_t) rx[2] << 8) | (int16_t) rx[1];
      p.flux[0] = (double) iflux * 1.515e-7;

      iflux = ((int16_t) rx[4] << 8) | (int16_t) rx[3];
      p.flux[1] = (double) iflux * 1.1515e-7;

      iflux = ((int16_t) rx[6] << 8) | (int16_t) rx[5];
      p.flux[2] = -(double) iflux * 1.1515e-7;

      rotate(p.flux);
      write((uint8_t *) &p, sizeof(p));
    }
  }
}

bool Ist8308::display()
{
  MagPacket p;
  char name[] = "Ist8308 (mag)";
  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.header);

    misc_printf("%10.3f %10.3f %10.3f uT   ", p.flux[0] * 1e6 + 10.9, p.flux[1] * 1e6 + 45.0, p.flux[2] * 1e6 - 37.5);
    misc_printf(" |                                       ");
    misc_printf(" |     N/A C |              | 0x%04X", p.header.status);
    if (p.header.status == STAT1_VAL_DRDY) misc_printf(" - OK\n");
    else misc_printf(" - NOK\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }

  return 0;
}

void Ist8308::register_callbacks(STM32H7Board & board, int32_t poll_phase_offset)
{
  if (async_bus_ == nullptr) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return;
  }

  board.callbacks().register_poll_client(this, poll_phase_offset);
  task_ = run();
}


