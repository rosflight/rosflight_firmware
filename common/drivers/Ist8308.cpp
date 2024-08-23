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

#include <Ist8308.h>
#include <misc.h>

#include <Time64.h>
extern Time64 time64;

DMA_RAM uint8_t ist8308_i2c_dma_buf[I2C_DMA_MAX_BUFFER_SIZE];
DTCM_RAM uint8_t ist8308_fifo_rx_buffer[IST8308_FIFO_BUFFERS * sizeof(MagPacket)];

#define WAI_REG 0x0
#define DEVICE_ID 0x08

#define STAT1_REG 0x10
#define STAT1_VAL_DRDY 0x1
#define STAT1_VAL_DOR 0x2

#define DATAX_L_REG 0x11
#define DATAX_H_REG 0x12
#define DATAY_L_REG 0x13
#define DATAY_H_REG 0x14
#define DATAZ_L_REG 0x15
#define DATAZ_H_REG 0x16

#define CNTL1_REG 0x30

#define CNTL2_REG 0x31
#define CNTL2_VAL_STANDBY_MODE 0x0
#define CNTL2_VAL_SINGLE_MODE 0x1
#define CNTL2_VAL_CONT_ODR10_MODE 0x2
#define CNTL2_VAL_CONT_ODR20_MODE 0x4
#define CNTL2_VAL_CONT_ODR50_MODE 0x6
#define CNTL2_VAL_CONT_ODR100_MODE 0x8
#define CNTL2_VAL_CONT_ODR200_MODE 0xA
#define CNTL2_VAL_CONT_ODR8_MODE 0xB
#define CNTL2_VAL_CONT_ODR1_MODE 0xC
#define CNTL2_VAL_CONT_ODR0P5_MODE 0xD
#define CNTL2_VAL_SINGLE_TEST_MODE 0x10

#define CNTL3_REG 0x32
#define CNTL3_VAL_SRST 1
#define CNTL3_VAL_DRDY_POLARITY_HIGH (1 << 2)
#define CNTL3_VAL_DRDY_EN (1 << 3)

#define CNTL4_REG 0x34
#define CNTL4_VAL_DYNAMIC_RANGE_500 0
#define CNTL4_VAL_DYNAMIC_RANGE_200 0x1

#define OSRCNTL_REG 0x41
#define OSRCNTL_VAL_XZ_1 (0)
#define OSRCNTL_VAL_XZ_2 (1)
#define OSRCNTL_VAL_XZ_4 (2)
#define OSRCNTL_VAL_XZ_8 (3)
#define OSRCNTL_VAL_XZ_16 (4)
#define OSRCNTL_VAL_XZ_32 (4)
#define OSRCNTL_VAL_Y_1 (0 << 3)
#define OSRCNTL_VAL_Y_2 (1 << 3)
#define OSRCNTL_VAL_Y_4 (2 << 3)
#define OSRCNTL_VAL_Y_8 (3 << 3)
#define OSRCNTL_VAL_Y_16 (4 << 3)
#define OSRCNTL_VAL_Y_32 (5 << 3)

uint32_t Ist8308::init(
  // Driver initializers
  uint16_t sample_rate_hz,
  // I2C initializers
  I2C_HandleTypeDef * hi2c, uint16_t i2c_address)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "-%s", "Ist8308");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  hi2c_ = hi2c;
  address_ = i2c_address << 1;
  launchUs_ = 0;
  // groupDelay_ = 0; //Computed later based on launchUs_ and drdy_ timestamps.

  rxFifo_.init(IST8308_FIFO_BUFFERS, sizeof(MagPacket), ist8308_fifo_rx_buffer);

  dtMs_ = 1000. / (double) sampleRateHz_;

  // Check if we are the right chip
  uint8_t reg = WAI_REG;
  if (HAL_I2C_Master_Transmit(hi2c_, address_, &reg, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  uint8_t device_id = 0;
  if (HAL_I2C_Master_Receive(hi2c_, address_, &device_id, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  misc_printf("IST8308 Device ID = 0x%02X (0x%02X) - ", device_id, DEVICE_ID);
  if (device_id == DEVICE_ID) {
    misc_printf("OK\n");
  } else {
    misc_printf("ERROR\n");
    {
      initializationStatus_ |= DRIVER_ID_MISMATCH;
      return initializationStatus_;
    }
  }

  uint8_t cmd[2];
  // Reset
  cmd[0] = CNTL3_REG;
  cmd[1] = CNTL3_VAL_SRST;
  if (HAL_I2C_Master_Transmit(hi2c_, address_, cmd, 2, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  HAL_Delay(20);

  // Write CNTL3_REG (to check status?)
  reg = CNTL3_REG;
  if (HAL_I2C_Master_Transmit(hi2c_, address_, &reg, 1, 1000) != HAL_OK) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  uint8_t cntl3 = 0;
  if (HAL_I2C_Master_Receive(hi2c_, address_, &cntl3, 1, 1000) != HAL_OK) {
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
  if (HAL_I2C_Master_Transmit(hi2c_, address_, cmd, 2, 1000) != HAL_OK) return DRIVER_HAL_ERROR;

  cmd[0] = OSRCNTL_REG;
  cmd[1] = OSRCNTL_VAL_Y_16 | OSRCNTL_VAL_XZ_16;
  if (HAL_I2C_Master_Transmit(hi2c_, address_, cmd, 2, 1000) != HAL_OK) return DRIVER_HAL_ERROR;

  cmd[0] = CNTL2_REG;
  cmd[1] = CNTL2_VAL_SINGLE_MODE;
  if (HAL_I2C_Master_Transmit(hi2c_, address_, cmd, 2, 1000) != HAL_OK) return DRIVER_HAL_ERROR;

  return initializationStatus_;
}

PollingState Ist8308::state(uint64_t poll_counter)
{
  uint32_t rollover = 10000; // us (100 Hz) 0-99 slots at 10 kHz
  PollingStateStruct lut[] = //
    {
      // at 10kHz, each count is 100us. I2C is 90us pre byte
      // Mag
      {0, IST8308_CMD}, // addr + 2 byte, so leave at least 3 counts
      {89, IST8308_TX}, // addr + 2 bytes
      {92, IST8308_RX}, // addr + 6 bytes (start at leat 84 counts after cmd
    };
  return PollingStateLookup(lut, sizeof(lut) / sizeof(PollingStateStruct),
                            poll_counter % (rollover / POLLING_PERIOD_US));
}

bool Ist8308::poll(uint64_t poll_counter)
{
  PollingState poll_state = state(poll_counter);
  if (poll_state == IST8308_CMD) {
    launchUs_ = time64.Us();
    ist8308_i2c_dma_buf[0] = CNTL2_REG;
    ist8308_i2c_dma_buf[1] = CNTL2_VAL_SINGLE_MODE;

    if ((dmaRunning_ = (HAL_OK == HAL_I2C_Master_Transmit_DMA(hi2c_, address_, ist8308_i2c_dma_buf, 2))))
      i2cState_ = poll_state;
    else i2cState_ = IST8308_ERROR;
  } else if (poll_state == IST8308_TX) // Write the register we want to read
  {
    drdy_ = time64.Us();
    ist8308_i2c_dma_buf[0] = STAT1_REG;
    if ((dmaRunning_ = (HAL_OK == HAL_I2C_Master_Transmit_DMA(hi2c_, address_, ist8308_i2c_dma_buf, 1))))
      i2cState_ = poll_state;
    else i2cState_ = IST8308_ERROR;
  } else if (poll_state == IST8308_RX) {
    if ((dmaRunning_ = (HAL_OK == HAL_I2C_Master_Receive_DMA(hi2c_, address_, ist8308_i2c_dma_buf, 7))))
      i2cState_ = poll_state;
    else i2cState_ = IST8308_ERROR;
  }
  return dmaRunning_;
}

void Ist8308::endDma(void)
{
  //	if(i2cState_ == IST8308_CMD) {} // do nothing
  //	if(i2cState_ == IST8308_TX) {}  // do nothing
  //  else
  if (i2cState_ == IST8308_RX) {
    MagPacket p;
    p.timestamp = time64.Us();
    p.drdy = drdy_;
    p.groupDelay = p.timestamp - (launchUs_ + drdy_) / 2;
    p.status = ist8308_i2c_dma_buf[0];
    p.temperature = 0;

    int16_t iflux = ((int16_t) ist8308_i2c_dma_buf[2] << 8) | (int16_t) ist8308_i2c_dma_buf[1];
    p.flux[0] = (double) iflux * 1.515e-7; // Tesla

    iflux = ((int16_t) ist8308_i2c_dma_buf[4] << 8) | (int16_t) ist8308_i2c_dma_buf[3];
    p.flux[1] = (double) iflux * 1.1515e-7; // Tesla

    iflux = ((int16_t) ist8308_i2c_dma_buf[6] << 8) | (int16_t) ist8308_i2c_dma_buf[5];
    p.flux[2] = -(double) iflux * 1.1515e-7; // Tesla

    if (p.status == STAT1_VAL_DRDY) rxFifo_.write((uint8_t *) &p, sizeof(p));
  }
  i2cState_ = IDLE_STATE;
  dmaRunning_ = false;
}
bool Ist8308::display()
{
  MagPacket p;
  char name[] = "Ist8308 (mag)";
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);

    misc_printf("%10.3f %10.3f %10.3f uT   ", p.flux[0] * 1e6 + 10.9, p.flux[1] * 1e6 + 45.0, p.flux[2] * 1e6 - 37.5);
    misc_printf(" |                                       ");
    misc_printf(" |     N/A C |              | 0x%04X", p.status);
    if (p.status == STAT1_VAL_DRDY) misc_printf(" - OK\n");
    else misc_printf(" - NOK\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }

  return 0;
}
