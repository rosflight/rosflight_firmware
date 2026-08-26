/**
 ******************************************************************************
 * File     : MS4525.cpp
 * Date     : May 28, 2024
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
#include "Ms4525.h"

#include "Time64.h"
#include "misc.h"
#include "stm32_h7.hpp"

extern Time64 time64;

#define MS4525_OK (0x0000)

DTCM_RAM uint8_t ms4525_double_buffer[2 * sizeof(PressurePacket)];

#define MS4525_I2C_DMA_SIZE (4)

#define ROLLOVER 10000
#define MS4525_CMDRXSTART 0
#define MS4525_CMDRX1 15
#define MS4525_CMDRX2 30
#define MS4525_CMDRX3 45
#define MS4525_CMDRX4 60
#define MS4525_CMDRXSEND 75

uint32_t Ms4525::init(uint16_t sample_rate_hz, I2C_HandleTypeDef * hi2c, uint16_t i2c_address)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Ms4525");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  address_ = i2c_address << 1;

  double_buffer_.init(ms4525_double_buffer, sizeof(ms4525_double_buffer));
  drdy_ = 0;

  // Read the status register
  uint8_t sensor_status[2];
  // Receive 1 bytes of data over I2C
  HAL_StatusTypeDef i2cstatus = HAL_I2C_Master_Receive(hi2c, address_, sensor_status, 2, 1000);

  misc_printf("MS4525 Status = 0x%02X - ", (sensor_status[0] >> 6) & 0x0003);
  if (i2cstatus == HAL_OK) misc_printf("OK\n");
  else {
    misc_printf("ERROR\n");
    initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;
  }

  misc_printf("\n");

  return initializationStatus_;
}

bool Ms4525::poll(uint64_t poll_counter)
{
  uint16_t poll_state;
  if (!stm32_h7_board.polling_timer().polling_state(poll_counter, ROLLOVER, poll_state)) return false;
  if (async_bus_ == nullptr) return false;

  poll_signal_.tick(poll_counter);
  if (poll_state == MS4525_CMDRXSTART) {
    drdy_ = time64.Us();
    poll_signal_.trigger();
  }
  return false;
}

AsyncTask<void> Ms4525::run()
{
  float pressure_filtered = 0.0f;
  uint8_t rx[MS4525_I2C_DMA_SIZE] = {};

  while (true) {
    co_await poll_signal_.wait_for_trigger();
    for (uint16_t sample = 0; sample < 6; sample++) {
      if (sample != 0U) {
        co_await poll_signal_.delay_ticks(MS4525_CMDRX1 - MS4525_CMDRXSTART);
      }

      if ((co_await async_bus_->read(address_, rx, MS4525_I2C_DMA_SIZE)).status != AsyncStatus::OK) {
        continue;
      }

      if ((rx[0] & 0xC0) == MS4525_OK) {
        uint32_t i_pressure = (uint32_t) (rx[0] & 0x3F) << 8 | (uint32_t) rx[1];
        static double pmax = 6894.76; // (=-pmin) Pa

        float pressure = (((double) i_pressure - 1638.3) / 6553.2 - 1.0) * pmax; // Pa

        // Anti-alias filter since we are reporting data at a lower rate.
        const float alpha = 0.5f;
        pressure_filtered = alpha * pressure + (1.0f - alpha) * pressure_filtered;

        if (sample == 5U) {
          PressurePacket p;
          p.pressure = pressure_filtered;
          uint32_t i_temperature = ((uint32_t) rx[2] << 3 | (uint32_t) (rx[3] & 0xE0) >> 5);
          p.temperature = (double) i_temperature * 200.0 / 2047.0 - 50.0 + 273.15; // K
          p.header.status = rx[0] & 0xC0;
          p.header.timestamp = drdy_;
          p.header.complete = time64.Us();
          write((uint8_t *) &p, sizeof(p));
        }
      }
    }
  }
}

bool Ms4525::display(void)
{
  PressurePacket p;
  char name[] = "MS4525 (pitot)";
  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.header);
    misc_printf("%10.3f Pa                          |                                        | "
                "%7.1f C |           "
                "   | 0x%04X",
                p.pressure, p.temperature - 273.15, p.header.status);
    if (p.header.status == MS4525_OK) misc_printf(" - OK\n");
    else misc_printf(" - NOK\n");

    return 1;
  } else {
    misc_printf("%s\n", name);
  }
  return true;
}

void Ms4525::register_callbacks(STM32H7Board & board, int32_t poll_phase_offset)
{
  if (async_bus_ == nullptr) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return;
  }

  board.callbacks().register_poll_client(this, poll_phase_offset);
  task_ = run();
}


