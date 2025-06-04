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
#include "Time64.h"
#include "misc.h"

extern Time64 time64;

//#define IIS2MDC_OK (0x0F)

#define WHO_AM_I 0x4F
#define CFG_REG_A 0x60
#define CFG_REG_B 0x61
#define CFG_REG_C 0x62
#define OFFSET_REG 0x45
#define STATUS_REG 0x67
#define OUT_FLUX 0x68
#define OUT_TEMP 0x6E

#define SPI_WRITE 0x00
#define SPI_READ 0x80

#define IIS_FLUX_CMD (STATUS_REG | SPI_READ) // read both status and flux
#define IIS_FLUX_BYTES 8
#define IIS_TEMP_CMD (OUT_TEMP | SPI_READ)
#define IIS_TEMP_BYTES 3

DMA_RAM uint8_t iis2mdc_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
DMA_RAM uint8_t iis2mdc_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];

DTCM_RAM uint8_t iis2mdc_fifo_rx_buffer[IIS2MDC_FIFO_BUFFERS * sizeof(MagPacket)];

#define ROLLOVER 10000
#define IIS2MDC_CMD 0
#define IIS2MDC_RX_H 97
#define IIS2MDC_RX_T (IIS2MDC_RX_H + 1)
#define IIS2MDC_IDLE_STATE 0xFFFF
#define IIS2MDC_STATE_ERROR 0xFFFF

uint32_t Iis2mdc::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
  uint16_t drdy_pin,                                 // Reset GPIO Pin
  // SPI initializers
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Chip Select GPIO Port
  uint16_t cs_pin                                   // Chip Select GPIO Pin
)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Iis2mdc");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;

  spi_.init(hspi, iis2mdc_dma_txbuf, iis2mdc_dma_rxbuf, cs_port, cs_pin);

  HAL_GPIO_WritePin(spi_.port_, spi_.pin_, GPIO_PIN_SET);

  rxFifo_.init(IIS2MDC_FIFO_BUFFERS, sizeof(MagPacket), iis2mdc_fifo_rx_buffer);

  spiState_ = IIS2MDC_IDLE_STATE;
  dmaRunning_ = false;

  //	uint8_t odr_mode=3;
  //	if( sampleRateHz_ <= 10) 		{sampleRateHz_ =  10; odr_mode = 0; }
  //	else if( sampleRateHz_ <= 20) 	{sampleRateHz_ =  20; odr_mode = 1; }
  //	else if( sampleRateHz_ <= 50) 	{sampleRateHz_ =  50; odr_mode = 2; }
  //	else if( sampleRateHz_ <= 100) 	{sampleRateHz_ = 100; odr_mode = 3; }
  //	else 			                {sampleRateHz_ = 100; odr_mode = 3; }

  uint8_t id = readRegister(WHO_AM_I);

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
  writeRegister(CFG_REG_A, 0x20); // soft reset
  time64.dUs(10);                 // Wait at least 5 us
  writeRegister(CFG_REG_A, 0x40); // reboot
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
  // writeRegister(CFG_REG_A,0x80|(odr_mode<<2)); // continuous mode
  writeRegister(CFG_REG_A, 0x83); // Set to idle

  // Register B (0x61)
  // [7:5] 000
  // [4]  1 OFF_CANC_ONE_SHOT 1=Offset Cancellation in single mode
  // [3]  0
  // [2]  0 Set Freq of Set pulse to 63 ODR
  // [1]  1, OFF_CANC 1= enable offset cancellation in single mode
  // [0]  0 LPF disable offset filter (1- enabled)
  writeRegister(CFG_REG_B, 0x12); // 0001 0010 For Single Mode (alternating set/reset)

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
  writeRegister(CFG_REG_C, 0x31); // 0011 0001 = 0x31 // 0011 1001 = 0x39

  // INT_CTRL_REG (0x63)
  // Disable Interrupts (this is not DRDY)
  writeRegister(0x63, 0x00);
  writeRegister(0x64, 0x00);
  writeRegister(0x65, 0x00);
  writeRegister(0x66, 0x00);

  // Read Status Register (0x67)
  uint8_t sensor_status = readRegister(STATUS_REG);
  misc_printf("IIS2MDC: Mag status register = 0x%02X (0x00)\n", sensor_status);
  if (sensor_status != 0x00) initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;

  // Read Offset Registers (6 bytes starting 0x45)
  uint8_t tx[7], h[7];
  memset(tx, 0, sizeof(tx));
  tx[0] = OFFSET_REG | SPI_READ;
  spi_.rx(tx, h, 7, 100);
  misc_printf("H Offsets should be zero %8d %8d %8d mGauss\n", ((int16_t) h[1] | (int16_t) h[2] << 8) * 3 / 2,
              ((int16_t) h[3] | (int16_t) h[4] << 8) * 3 / 2, ((int16_t) h[5] | (int16_t) h[6] << 8) * 3 / 2);

  return initializationStatus_;
}

bool Iis2mdc::poll(uint64_t poll_counter)
{
  PollingState poll_state = (PollingState) (poll_counter % (ROLLOVER / POLLING_PERIOD_US));

  // Start measurement sequence
  if (poll_state == IIS2MDC_CMD) // Command Pressure Read
  {
    launchUs_ = time64.Us();
    uint8_t cmd[2] = {CFG_REG_A | SPI_WRITE, 0x81}; // CFG_REG_A
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(cmd, 2)))) spiState_ = poll_state;
    else spiState_ = IIS2MDC_STATE_ERROR;
  }
  // Read Status & Flux
  else if (poll_state == IIS2MDC_RX_H) {
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(IIS_FLUX_CMD, IIS_FLUX_BYTES)))) spiState_ = poll_state;
    else spiState_ = IIS2MDC_STATE_ERROR;
  }
  // Read Temperature. Chip does not autoincrement into the temperature register, so we have to do separate reads for
  // flux and temperature
  else if (poll_state == IIS2MDC_RX_T) {
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(IIS_TEMP_CMD, IIS_TEMP_BYTES)))) spiState_ = poll_state;
    else spiState_ = IIS2MDC_STATE_ERROR;
  }
  return false;
}

void Iis2mdc::endDma(void)
{
  uint8_t * rx = spi_.endDma();
  static MagPacket p;
  static int16_t previous_data[3] = {0};
  if (spiState_ == IIS2MDC_RX_H) // Flux Data and DRDY
  {
    memset(&p, 0, sizeof(p)); // clear p
    p.drdy = time64.Us();
    p.header.status = rx[1];

    int16_t data = (rx[3] << 8) | rx[2];
    p.flux[0] = -(data + previous_data[0]) / 2. * 1.5e-7; // T, 1.5e-7 T/LSB, 1mG = 1e-7 T.
    previous_data[0] = data;

    data = (rx[5] << 8) | rx[4];
    p.flux[1] = (data + previous_data[1]) / 2. * 1.5e-7; // T, 1.5e-7 T/LSB
    previous_data[1] = data;

    data = (rx[7] << 8) | rx[6];
    p.flux[2] = (data + previous_data[2]) / 2. * 1.5e-7; // T, 1.5e-7 T/LSB
    previous_data[2] = data;

  } else if (spiState_ == IIS2MDC_RX_T) {
    int16_t data = (rx[2] << 8) | rx[1];
    p.temperature = (double) data / 8.0 + 25.0 + 273.15; // K

    p.header.timestamp = time64.Us();
    // 5 ms is added to group delay because of the set/reset averaging used in the flux measurement above.
    p.groupDelay = p.header.timestamp - (p.drdy + launchUs_) / 2 + 5000;
    if (p.header.status == IIS2MDC_OK) rxFifo_.write((uint8_t *) &p, sizeof(p));
  }

  spiState_ = IIS2MDC_IDLE_STATE;
  dmaRunning_ = false;
}

bool Iis2mdc::display()
{
  MagPacket p;
  char name[] = "Iis2mdc (mag)";
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {

    float total_flux = sqrt(p.flux[0] * p.flux[0] + p.flux[1] * p.flux[1] + p.flux[2] * p.flux[2]);

    misc_header(name, p.drdy, p.header.timestamp, p.groupDelay);

    misc_f32(NAN, NAN, p.flux[0] * 1e6, "hx", "%6.2f", "uT");
    misc_f32(NAN, NAN, p.flux[1] * 1e6, "hy", "%6.2f", "uT");
    misc_f32(NAN, NAN, p.flux[2] * 1e6, "hz", "%6.2f", "uT");

    misc_f32(20, 70, total_flux * 1e6, "|h|", "%6.2f", "uT");
    misc_f32(18, 50, p.temperature - 273.15, "Temp", "%5.1f", "C");
    misc_x16(IIS2MDC_OK, p.header.status, "Status");
    misc_printf("\n");
    return 1;
  } else {
    misc_printf("%s\n", name);
  }

  return 0;
}

void Iis2mdc::writeRegister(uint8_t address, uint8_t value)
{
  uint8_t tx[2] = {0};
  tx[0] = (address) | SPI_WRITE;
  tx[1] = value;
  spi_.tx(tx, 2, 100);
}

uint8_t Iis2mdc::readRegister(uint8_t address)
{
  uint8_t tx[2] = {0};
  uint8_t rx[2] = {0};
  tx[0] = (address) | SPI_READ;
  tx[1] = 0;
  HAL_StatusTypeDef hal_status = spi_.rx(tx, rx, 2, 100);
  return rx[1] | hal_status;
}
