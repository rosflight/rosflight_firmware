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
#include <Packets.h>
#include <Time64.h>
#include <misc.h>

#define DPS310_OK (0xE0D0)

#define DPS310_CONTINUOUS_MODE false

#define SPI_WRITE ((uint8_t) 0x00)
#define SPI_READ ((uint8_t) 0x80)

//#define DPS310_READ_T_CMD		(0x03|SPI_READ)
//#define	DPS310_READ_T_BUFFBYTES 	(4)
//
//#define DPS310_READ_P_CMD		(0x00|SPI_READ)
//#define	DPS310_READ_P_BUFFBYTES 	(4)

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

DMA_RAM uint8_t dps310_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
DMA_RAM uint8_t dps310_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];

DTCM_RAM uint8_t dps310_fifo_rx_buffer[DPS310_FIFO_BUFFERS * sizeof(PressurePacket)];

static int32_t Compliment(int32_t x, int16_t bits)
{
  if (x & ((int32_t) 1 << (bits - 1))) { x -= (int32_t) 1 << bits; }
  return x;
}

PollingState Dps310::state(uint64_t poll_counter)
{
  uint32_t rollover = 20000; // us (50 Hz) 0-199 slots at 10 kHz
  PollingStateStruct lut[] = // BARO at 50 Hz
    {
      {0, DPS310_CMD_P},    // at 10kHz, each count is 100us.
      {145, DPS310_DRDY_P}, //
      {146, DPS310_RX_P},   //
      {147, DPS310_CMD_T},  //
      {177, DPS310_DRDY_T}, //
      {178, DPS310_RX_T},
    };
  return PollingStateLookup(lut, sizeof(lut) / sizeof(PollingStateStruct),
                            poll_counter % (rollover / POLLING_PERIOD_US));
}

uint32_t Dps310::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
  uint16_t drdy_pin,                                 // Reset GPIO Pin
  // SPI initializers
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Chip Select GPIO Port
  uint16_t cs_pin,                                  // Chip Select GPIO Pin
  // Mode
  bool three_wire)
{
  uint32_t status = DRIVER_OK;

  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;

  spi_.init(hspi, dps310_dma_txbuf, dps310_dma_rxbuf, cs_port, cs_pin);
  spiState_ = IDLE_STATE;
  dmaRunning_ = false;

  timeoutMs_ = 100;
  // groupDelay_		= 1000000/sampleRateHz_;
  HAL_GPIO_WritePin(spi_.port_, spi_.pin_, GPIO_PIN_SET);

  rxFifo_.init(DPS310_FIFO_BUFFERS, sizeof(PressurePacket), dps310_fifo_rx_buffer);

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
  if (three_wire) writeRegister(CFG_REG, 0x01);
  else writeRegister(CFG_REG, 0x00);

    // Product ID 0x0D
#define PRODUCT_ID 0x0D
  uint8_t product_id = readRegister(PRODUCT_ID);
  misc_printf("DPS310: PRODUCT ID = 0x%02X  (0x10) -", product_id);
  if (product_id == 0x10) misc_printf(" OK\n");
  else {
    status |= DRIVER_ID_MISMATCH;
    misc_printf(" Not OK\n");
  }

  // Calibration constants
#define MEAS_CFG 0x08
  uint8_t coef_rdy = readRegister(MEAS_CFG) & 0x80;

  for (int n = 0; n < 10; n++) // Wait 10 times for Coefficients to be ready
  {
    coef_rdy = readRegister(MEAS_CFG) & 0x80;
    //		misc_printf("DPS310: COEF_RDY   = 0x%02X\n",coef_rdy);
    if ((coef_rdy & 0x80) == 0x80) break;
    time64.dUs(1000);
  }
  misc_printf("DPS310: COEF_RDY = 0x%02X (0x80) ", coef_rdy);
  if ((coef_rdy & 0x80) == 0x80) misc_printf("- READY\n");
  else {
    misc_printf("- NOT READYn\n");
    status |= DRIVER_SELF_DIAG_ERROR;
  }

  misc_printf("DPS310: Reading Coefficients\n");

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

  misc_printf("DPS310: C0,  C1  = %10.0f %10.0f\n", C0_, C1_);
  misc_printf("DPS310: C00, C10 = %10.0f %10.0f\n", C00_, C10_);
  misc_printf("DPS310: C01, C11 = %10.0f %10.0f\n", C01_, C11_);
  misc_printf("DPS310: C20, C21 = %10.0f %10.0f\n", C20_, C21_);
  misc_printf("DPS310: C30      = %10.0f\n", C30_);

#define COEF_SRCE 0x28
  uint8_t temp_source = readRegister(COEF_SRCE) & 0x80;
  misc_printf("DPS310: temp source = 0x%02X\n", temp_source);

#define PRS_CFG 0x06            // Pressure Configuration
  writeRegister(PRS_CFG, 0x63); // 64 measurements per second, 8x oversampling

#define TMP_CFG 0x07                          // Temperature Configuration
  writeRegister(TMP_CFG, temp_source | 0x60); // 64 measurements per second, no oversampling

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
#define CFG_REG 0x09
  if (three_wire) {
#if DPS310_CONTINUOUS_MODE
    writeRegister(
      CFG_REG,
      0x91); // Interrupt on T only 3-wire supports interrupts, 4-wire does not support interrupts
#else
    writeRegister(CFG_REG, 0xB1); // Interrupt on both P and T
#endif
  } else {
#if DPS310_CONTINUOUS_MODE
    writeRegister(
      CFG_REG,
      0x90); // Interrupt on T only 3-wire supports interrupts, 4-wire does not support interrupts
#else
    writeRegister(CFG_REG, 0xB0); // Interrupt on both P and T
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
#define MEAS_CFG 0x08
#if DPS310_CONTINUOUS_MODE
  writeRegister(MEAS_CFG, 0x07); // Start background measurement
#endif
  return status;
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

bool Dps310::poll(uint64_t poll_counter)
{
  PollingState poll_state = state(poll_counter);

  // Start P measurement sequence
  if (poll_state == DPS310_CMD_P) // Command Pressure Read
  {
    uint8_t cmd[2] = {MEAS_CFG | SPI_WRITE, 0x01};
    launchUs_ = time64.Us();
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(cmd, 2)))) spiState_ = DPS310_CMD_P;
    else spiState_ = DPS310_ERROR;
  }
  // Get P DRDY
  else if (poll_state == DPS310_DRDY_P) {
    uint8_t cmd[2] = {MEAS_CFG | SPI_READ, 0};
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(cmd, 2)))) spiState_ = poll_state;
    else spiState_ = DPS310_ERROR;
  }
  // Read P data
  else if (poll_state == DPS310_RX_P) // Start DMA read of Temperature Data 2.41ms after start
  {
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(DPS310_READ_P_CMD, DPS310_READ_P_BUFFBYTES))))
      spiState_ = poll_state;
    else spiState_ = DPS310_ERROR;
  }
  // Start T measurement sequence
  else if (poll_state == DPS310_CMD_T) // Command Temperature Daq
  {
    uint8_t cmd[2] = {MEAS_CFG | SPI_WRITE, 0x02}; // Temperature
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(cmd, 2)))) spiState_ = poll_state;
    else spiState_ = DPS310_ERROR;
  }
  // Get T DRDY
  else if (poll_state == DPS310_DRDY_T) {
    uint8_t cmd[2] = {MEAS_CFG | SPI_READ, 0};
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(cmd, 2)))) spiState_ = poll_state;
    else spiState_ = DPS310_ERROR;
  }
  // Read T data
  else if (poll_state == DPS310_RX_T) // Start DMA read of Pressure Data
  {
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(DPS310_READ_T_CMD, DPS310_READ_T_BUFFBYTES))))
      spiState_ = poll_state;
    else spiState_ = DPS310_ERROR;
  }
  return dmaRunning_;
}

void Dps310::endDma(void)
{
  uint8_t * rx = spi_.endDma();
  static double Traw;
  static PressurePacket p;

  if (spiState_ == DPS310_DRDY_P) // Pressure DRDY
  {
    if (rx[1] & 0x10) {
      p.status |= (uint16_t) rx[1];
      p.drdy = time64.Us();
    }
  } else if (spiState_ == DPS310_DRDY_T) // Temperature DRDY
  {
    if (rx[1] & 0x20) p.status = (uint16_t) rx[1] << 8;
  } else if (spiState_ == DPS310_RX_T) // Temperature Data
  {
    int32_t traw = ((int32_t) rx[4] << 24 | (int32_t) rx[5] << 16 | (int32_t) rx[6] << 8) >> 8;
    Traw = (double) traw / KT;
    p.temperature = C0_ * 0.5 + C1_ * Traw + 273.15; // K
  } else if (spiState_ == DPS310_RX_P)               // Pressure Data
  {
    int32_t praw = ((int32_t) rx[1] << 24 | (int32_t) rx[2] << 16 | (int32_t) rx[3] << 8) >> 8;
    double Praw = (double) praw / KP;
    p.pressure = C00_ + Praw * (C10_ + Praw * (C20_ + Praw * C30_))
      + Traw * (C01_ + Praw * (C11_ + Praw * C21_)); // Pa

    p.timestamp = time64.Us();
    p.groupDelay = p.timestamp - (p.drdy + launchUs_) / 2;
    if (p.status == DPS310_OK) rxFifo_.write((uint8_t *) &p, sizeof(p));
    p.status = 0;
  }

  spiState_ = IDLE_STATE;
  dmaRunning_ = false;
}

bool Dps310::display(void)
{
  PressurePacket p;
  char name[] = "Dps310 (baro)";
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f kPa                         |                                        | "
                "%7.1f C |           "
                "   | 0x%04X",
                p.pressure / 1000., p.temperature - 273.15, p.status);
    if (p.status == DPS310_OK) misc_printf(" - OK\n");
    else misc_printf(" - NOK\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }
  return 0;
}
