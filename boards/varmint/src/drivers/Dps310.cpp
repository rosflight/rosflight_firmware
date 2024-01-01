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

#include <Dps310.h>
#include <misc.h>

#define SPI_WRITE ((uint8_t)0x00)
#define SPI_READ ((uint8_t)0x80)

#define DPS310_READ_CMD (0x00 | SPI_READ)
#define DPS310_BUFFBYTES (12)

#define KP 7864320.0 // 8x oversample
#define KT 524288.0  // No oversampling
//	#define KP1  524288.0 // 2 times
//	#define KP2 1572864.0 // 2 times
//	#define KP4 3670016.0 // 4 times
//	#define KP8 7864320.0 // 8 times
//   etc.

extern Time64 time64;

__attribute__((section("my_dma_buffers")))
__attribute__((aligned(32))) static uint8_t dps310_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE] = {0};
__attribute__((section("my_dma_buffers")))
__attribute__((aligned(32))) static uint8_t dps310_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE] = {0};

__attribute__((section("my_buffers")))
__attribute__((aligned(32))) static uint8_t dps310_fifo_rx_buffer[DPS310_FIFO_BUFFERS * sizeof(BaroPacket)] = {0};

static int32_t Compliment(int32_t x, int16_t bits)
{
  if (x & ((int32_t)1 << (bits - 1)))
  {
    x -= (int32_t)1 << bits;
  }
  return x;
}
uint32_t Dps310::init(

    // Driver initializers
    uint16_t sample_rate_hz,
    GPIO_TypeDef *drdy_port, // Reset GPIO Port
    uint16_t drdy_pin,       // Reset GPIO Pin
    // SPI initializers
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port, // Chip Select GPIO Port
    uint16_t cs_pin        // Chip Select GPIO Pin
)
{
  uint32_t status = DRIVER_OK;

  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;

  spi_.init(hspi, dps310_dma_txbuf, dps310_dma_rxbuf, cs_port, cs_pin);

  timeoutMs_ = 100;
  groupDelay_ = 1000000 / sampleRateHz_;
  HAL_GPIO_WritePin(spi_.port_, spi_.pin_, GPIO_PIN_SET);

  rxFifo_.init(DPS310_FIFO_BUFFERS, sizeof(BaroPacket), dps310_fifo_rx_buffer);

#define RESET 0x0C
  writeRegister(RESET, 0x09);
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
  writeRegister(CFG_REG, 0x01);

// Product ID 0x0D
#define PRODUCT_ID 0x0D
  uint8_t product_id = readRegister(PRODUCT_ID);
  misc_printf("DPS310: PRODUCT ID = 0x%02X  (0x10) -", product_id);
  if (product_id == 0x10)
    misc_printf(" OK\n\r");
  else
  {
    status |= DRIVER_ID_MISMATCH;
    misc_printf(" Not OK\n\r");
  }

// Calibration constants
#define MEAS_CFG 0x08
  uint8_t coef_rdy = readRegister(MEAS_CFG) & 0x80;

  for (int n = 0; n < 10; n++) // Wait 10 times for Coefficients to be ready
  {
    coef_rdy = readRegister(MEAS_CFG) & 0x80;
    //		misc_printf("DPS310: COEF_RDY   = 0x%02X\n\r",coef_rdy);
    if ((coef_rdy & 0x80) == 0x80)
      break;
    time64.dUs(1000);
  }
  misc_printf("DPS310: COEF_RDY = 0x%02X (0x80) ", coef_rdy);
  if ((coef_rdy & 0x80) == 0x80)
    misc_printf("- READY\n\r");
  else
  {
    misc_printf("- NOT READYn\n\r");
    status |= DRIVER_SELF_DIAG_ERROR;
  }

  misc_printf("DPS310: Reading Coefficients\n\r");

// Read Calibration Constants
#define COEF_REG 0x10
  uint8_t tx[19] = {COEF_REG | SPI_READ, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t rx[19];

  spi_.rx(tx, rx, 19, timeoutMs_);

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

  misc_printf("DPS310: C0,  C1  = %10.0f %10.0f\n\r", C0_, C1_);
  misc_printf("DPS310: C00, C10 = %10.0f %10.0f\n\r", C00_, C10_);
  misc_printf("DPS310: C01, C11 = %10.0f %10.0f\n\r", C01_, C11_);
  misc_printf("DPS310: C20, C21 = %10.0f %10.0f\n\r", C20_, C21_);
  misc_printf("DPS310: C30      = %10.0f\n\r", C30_);

#define COEF_SRCE 0x28
  uint8_t temp_source = readRegister(COEF_SRCE) & 0x80;
  misc_printf("DPS310: temp source = 0x%02X\n\r", temp_source);

#define PRS_CFG 0x06 // Pressure Configuration
  // 64 measurements per second, 8x oversampling
  writeRegister(PRS_CFG, 0x63); // write_reg(PRS_CFG, 0);

#define TMP_CFG 0x07 // Temperature Configuration
  // 64 measurements per second, no oversampling
  writeRegister(TMP_CFG, temp_source | 0x60);

// Interrupt and FIFO Config 0x09
// 7 - 	1, DRDY active high
// 6 - 	0, Disable FIFO full interrupt
// 5 - 	0, Int on temp
// 4 - 	1, Int on pressure
// 3 - 	0, no Temp data shift
// 2 - 	0, no Press data shift
// 1 - 	0, Disable FIFO
// 0 - 	1, 3-wire SPI interface
// 1011 0001 = 0xB1 active high
// 0011 0001 = 0x31 active low
// 1001 0001 = 0x91
#define CFG_REG 0x09
  writeRegister(CFG_REG, 0x91);

// Measurement Configuration
// 7 - 	0, read only
// 6 - 	0, read only
// 5 - 	0, read only
// 4 - 	0, read only
// 3 - 	0, reserved
// 2:0 - 	111, pressure and temperature continuous mode
// 0000 0111 =  0x07
#define MEAS_CFG 0x08
  writeRegister(MEAS_CFG, 0x07);

  return status;
}

bool Dps310::poll(void)
{
  static bool previous_drdy = 0;
  bool status = true;
  bool current_drdy = HAL_GPIO_ReadPin(drdyPort_, drdyPin_);

  // This sensor runs in continuous mode, so no action needed to instigate.

  if (!previous_drdy && current_drdy)
  {
    status = startDma();
  }
  previous_drdy = current_drdy;
  return status;
}

bool Dps310::startDma(void) // called to start dma read
{
  drdy_ = time64.Us();
  HAL_StatusTypeDef hal_Status = HAL_OK;
  hal_Status = spi_.startDma(DPS310_READ_CMD, DPS310_BUFFBYTES);
  return hal_Status == HAL_OK;
}

void Dps310::endDma(void)
{
  uint8_t *rx = spi_.endDma();

  BaroPacket p;

  p.drdy = drdy_;
  p.groupDelay = groupDelay_;
  p.status = (uint16_t)rx[9];

  int32_t traw = ((int32_t)rx[4] << 24 | (int32_t)rx[5] << 16 | (int32_t)rx[6] << 8) >> 8;

  double Traw = (double)traw / KT;
  p.temperature = C0_ * 0.5 + C1_ * Traw + 273.15; // K

  int32_t praw = ((int32_t)rx[1] << 24 | (int32_t)rx[2] << 16 | (int32_t)rx[3] << 8) >> 8;
  double Praw = (double)praw / KP;
  p.pressure = C00_ + Praw * (C10_ + Praw * (C20_ + Praw * C30_)) + Traw * (C01_ + Praw * (C11_ + Praw * C21_)); // Pa

  p.timestamp = time64.Us();
  rxFifo_.write((uint8_t *)&p, sizeof(p));
}

void Dps310::writeRegister(uint8_t address, uint8_t value)
{
  uint8_t tx[2] = {0};
  tx[0] = (address) | SPI_WRITE;
  tx[1] = value;
  spi_.tx(tx, 2, timeoutMs_);
}

uint8_t Dps310::readRegister(uint8_t address)
{
  uint8_t tx[2] = {0};
  uint8_t rx[2] = {0};
  tx[0] = (address) | SPI_READ;
  tx[1] = 0;
  spi_.rx(tx, rx, 2, timeoutMs_);
  return rx[1];
}

bool Dps310::display(void)
{
  BaroPacket p;
  char name[] = "Dps310 (baro)";
  if (rxFifo_.readMostRecent((uint8_t *)&p, sizeof(p)))
  {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf(
        "%10.3f kPa                         |                                        | %7.1f C |              | "
        "0x%04X\n\r",
        p.pressure / 1000., p.temperature - 273.15, p.status);
    return 1;
  }
  else
  {
    misc_printf("%s\n\r", name);
  }
  return 0;
}
