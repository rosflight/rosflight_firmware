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

extern Time64 time64;

#define MS4525_OK (0x0000)

DMA_RAM uint8_t ms4525_i2c_dma_buf[I2C_DMA_MAX_BUFFER_SIZE];
DTCM_RAM uint8_t ms4525_signal_rx_buffer[2 * sizeof(PressurePacket)];

#define MS4525_I2C_DMA_SIZE (4)

#define ROLLOVER 10000
#define MS4525_CMDRXSTART 0
#define MS4525_CMDRX1 15
#define MS4525_CMDRX2 30
#define MS4525_CMDRX3 45
#define MS4525_CMDRX4 60
#define MS4525_CMDRXSEND 75
#define MS4525_IDLE_STATE 0xFFFE
#define MS4525_STATE_ERROR 0xFFFF

uint32_t Ms4525::init(
  // Driver initializers
  uint16_t sample_rate_hz,
  // I2C initializers
  I2C_HandleTypeDef * hi2c, // The SPI handle
  uint16_t i2c_address      // Chip select Port
)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Ms4525");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  hi2c_ = hi2c;
  address_ = i2c_address << 1;

  signal_.init(ms4525_signal_rx_buffer, sizeof(ms4525_signal_rx_buffer));

  dtMs_ = 1000. / (double) sampleRateHz_;
  drdy_ = 0;

  // Read the status register
  uint8_t sensor_status[2];
  // Receive 1 bytes of data over I2C
  HAL_StatusTypeDef i2cstatus = HAL_I2C_Master_Receive(hi2c_, address_, sensor_status, 2, 1000);

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
  PollingState poll_state = (PollingState) (poll_counter % (ROLLOVER / POLLING_PERIOD_US));
  if ((poll_state == MS4525_CMDRXSTART) || (poll_state == MS4525_CMDRX1) || (poll_state == MS4525_CMDRX2)
      || (poll_state == MS4525_CMDRX3) || (poll_state == MS4525_CMDRX4) || (poll_state == MS4525_CMDRXSEND)) {
    drdy_ = time64.Us();
    if (HAL_OK == HAL_I2C_Master_Receive_DMA(hi2c_, address_, ms4525_i2c_dma_buf, MS4525_I2C_DMA_SIZE)) // Receive 7 bytes of data over I2C
      i2cState_ = poll_state;
    else i2cState_ = MS4525_STATE_ERROR;
  }
  return false;
}

void Ms4525::endDma(void)
{
  static float pressure_filtered = 0;

  if ((i2cState_ == MS4525_CMDRXSTART) || (i2cState_ == MS4525_CMDRX1) || (i2cState_ == MS4525_CMDRX2)
      || (i2cState_ == MS4525_CMDRX3) || (i2cState_ == MS4525_CMDRX4) || (i2cState_ == MS4525_CMDRXSEND)) {
    if ((ms4525_i2c_dma_buf[0] & 0xC0) == MS4525_OK) {
      uint32_t i_pressure = (uint32_t) (ms4525_i2c_dma_buf[0] & 0x3F) << 8 | (uint32_t) ms4525_i2c_dma_buf[1];

      static double pmax = 6894.76; // (=-pmin) Pa

      float pressure = (((double) i_pressure - 1638.3) / 6553.2 - 1.0) * pmax; // Pa

      // Anti-alias filter since we are reporting data at a lower rate.
      const float alpha = 0.5;
      pressure_filtered = alpha * pressure + (1.0 - alpha) * pressure_filtered;

      if (i2cState_ == MS4525_CMDRXSEND) {
        PressurePacket p;

        p.pressure = pressure_filtered;

        uint32_t i_temperature =
          ((uint32_t) ms4525_i2c_dma_buf[2] << 3 | (uint32_t) (ms4525_i2c_dma_buf[3] & 0xE0) >> 5);
        p.temperature = (double) i_temperature * 200.0 / 2047.0 - 50.0 + 273.15; // K
        p.header.status = ms4525_i2c_dma_buf[0] & 0xC0;

        p.header.timestamp = drdy_;
        p.read_complete = time64.Us();

        write((uint8_t *) &p, sizeof(p));
      }
    }
  }

  i2cState_ = MS4525_IDLE_STATE;
}

bool Ms4525::display(void)
{
  PressurePacket p;
  char name[] = "MS4525 (pitot)";
  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.header.timestamp, p.read_complete );
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
