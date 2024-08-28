/**
 ******************************************************************************
 * File     : Auav.cpp
 * Date     : Mar 5, 2023
 ******************************************************************************
 *
 * Copyright (c) 2024, AeroVironment, Inc.
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
#include <Auav.h>
#include <Packets.h>
#include <Polling.h>
#include <Time64.h>
#include <misc.h>

extern Time64 time64;

#define AUAV_READ_BYTES 7

DMA_RAM uint8_t auav_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
DMA_RAM uint8_t auav_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];
DTCM_RAM uint8_t auav_pitot_fifo_rx_buffer[AUAV_PITOT_FIFO_BUFFERS * sizeof(PressurePacket)];
DTCM_RAM uint8_t auav_baro_fifo_rx_buffer[AUAV_BARO_FIFO_BUFFERS * sizeof(PressurePacket)];

uint32_t Auav::init(
  // Driver initializers
  uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
  uint16_t drdy_pin,                                 // Reset GPIO Pin
  // SPI initializers
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Chip Select GPIO Port
  uint16_t cs_pin,                                  // Chip Select GPIO Pin
  auav_press type)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Auav");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = drdy_port;
  drdyPin_ = drdy_pin;
  launchUs_ = 0;
  type_ = type;

  spi_.init(hspi, auav_dma_txbuf, auav_dma_rxbuf, cs_port, cs_pin);

  // Read calibration constants
  int32_t i32A = 0, i32B = 0, i32C = 0, i32D = 0, i32TC50HLE = 0;
  int8_t i8TC50H = 0, i8TC50L = 0, i8Es = 0;
  int8_t addr = 0, sensor_status_ready = 0;

  memset(name_, '\0', sizeof(name_));

  // Vent (zero) pressure is at 0.1 *2^24 nominal output for Gauge Sensor

  // Time to read vs. cmd_byte_
  //          Pitot (ms) Baro (ms)
  //          ---------- ---------
  // 0xAA   	  1.88       4.76 (single)
  // 0xAB      11.06     127.02 (repeating)
  // 0xAC       3.63      _9.33_ (avg 2)
  // 0xAD      _7.14_     18.43 (avg 4)
  // 0xAE      14.14      36.66 (avg 8)
  // 0xAF      28.16      73.14 (avg 16)
  // Start Measurement
  // Send 0xAD 0x00 0x00

  if (type_ == AUAV_PITOT) {
    rxFifo_.init(AUAV_PITOT_FIFO_BUFFERS, sizeof(PressurePacket), auav_pitot_fifo_rx_buffer);
    char name[] = "Auav (pitot)";
    strcpy(name_, name);
    memset(cmdBytes_, 0, AUAV_CMD_BYTES);
    cmdBytes_[0] = 0xAD; // ~100 Hz

    // Vent (zero) pressure is at 0.1 *2^24 nominal output for Gauge Sensor
    osDig_ = 0.1;  // *2^24;
    fss_ = 2488.4; // Pa
    off_ = 0.0;    // Pa
    addr = 43;
    // Status register:
    // 7 0
    // 6 Powered?
    // 5 Busy?
    // 4:3 Mode, 00 = command, 01 = cyclic 10 = idle/sleep
    // 2 Memory Error?
    // 1 Connection Check Fault?
    // 0 Math Saturation (out of range)
    // 0x50 is Powered & Idle/Sleep == Ready.
    sensor_status_ready_ = 0x50;
  } else // baro
  {
    rxFifo_.init(AUAV_BARO_FIFO_BUFFERS, sizeof(PressurePacket), auav_baro_fifo_rx_buffer);
    char name[] = "Auav (baro)";
    strcpy(name_, name);

    cmdBytes_[0] = 0xAC; // ~100 Hz

    // Vent (zero) pressure is at 0.1 *2^24 nominal output for Gauge Sensor
    osDig_ = 0.1;    // *2^24;
    fss_ = 100000.0; // Pa
    off_ = 25000.0;  // Pa
    addr = 47;
    // Status register:
    // 7 always 0
    // 6 Powered?
    // 5 Busy?
    // 4:3 Mode, 00 = Normal Operation
    // 2 Memory Error?
    // 1 Sensor Configuration, always 0
    // 0 ALU error (out of range)
    // 0x40 is Powered & Normal Operation == Ready
    sensor_status_ready_ = 0x40;
  }
  // Read Status
  uint8_t tx = 0xF0, sensor_status = 0;
  spi_.rx(&tx, &sensor_status, 1, 100);
  misc_printf("AUAV Status = 0x%02X (0x%02X) - ", sensor_status, sensor_status_ready);
  if (sensor_status == sensor_status_ready) misc_printf("OK\n");
  else {
    misc_printf("ERROR\n");
    initializationStatus_ |= DRIVER_SELF_DIAG_ERROR;
  }

  // These i32 Reads return 2 register values merged as int32
  // i32 then normalized to +/- 1.0
  // Note that Diff data block is shifted 4 down from ABS locations
  i32A = readCfg(addr, &spi_);
  i32B = readCfg(addr + 2, &spi_);
  i32C = readCfg(addr + 4, &spi_);
  i32D = readCfg(addr + 6, &spi_);
  i32TC50HLE = readCfg(addr + 8, &spi_);

  LIN_A_ = ((double) (i32A)) / ((double) (0x7FFFFFFF));
  LIN_B_ = (double) (i32B) / (double) (0x7FFFFFFF);
  LIN_C_ = (double) (i32C) / (double) (0x7FFFFFFF);
  LIN_D_ = (double) (i32D) / (double) (0x7FFFFFFF);

  i8TC50H = (i32TC50HLE >> 24) & 0xFF;
  i8TC50L = (i32TC50HLE >> 16) & 0xFF;
  i8Es = (i32TC50HLE) &0xFF;

  Es_ = (double) (i8Es) / (double) (0x7F);       // norm to +/- 1.0
  TC50H_ = (double) (i8TC50H) / (double) (0x7F); // norm to +/- 1.0
  TC50L_ = (double) (i8TC50L) / (double) (0x7F); // norm to +/- 1.0

  misc_printf("\n");

  return initializationStatus_;
}

uint32_t Auav::readCfg(uint8_t address, Spi * spi)
{
  // First word
  uint8_t tx[3] = {0}, hi[3] = {0}, lo[3] = {0};

  tx[0] = address;
  spi->tx(tx, 3, 100);
  time64.dUs(20); // 20 us for device to respond
  tx[0] = 0xF0;
  spi->rx(tx, hi, 3, 100);
  time64.dUs(20); // don't know if this is needed.

  // Second word
  tx[0] = address + 1;
  spi->tx(tx, 3, 100);
  time64.dUs(20); // 20 us for device to respond
  tx[0] = 0xF0;
  spi->rx(tx, lo, 3, 100);
  time64.dUs(20); // don't know if this is needed.

  uint32_t value = (uint32_t) hi[1] << 24 | (uint32_t) hi[2] << 16 | ((uint32_t) lo[1]) << 8 | (uint32_t) lo[2];

  return value;
}

PollingState Auav::state(uint64_t poll_counter)
{
  uint32_t rollover = 10000; // us (100 Hz) 0-99 slots at 10 kHz
  PollingStateStruct lut[] = //
    {
      // Pitot
      {1, AUAV_PITOT_CMD},
      {74, AUAV_PITOT_RX}, // 7.2ms
                              // Baro
      {0, AUAV_BARO_CMD},
      {73, AUAV_BARO_RX}, // 9.4ms
    };
  return PollingStateLookup(lut, sizeof(lut) / sizeof(PollingStateStruct),
                            poll_counter % (rollover / POLLING_PERIOD_US));
}

bool Auav::poll(uint64_t poll_counter)
{
  PollingState poll_state = state(poll_counter);

  if ((poll_state == AUAV_PITOT_CMD && type_ == AUAV_PITOT)
      || (poll_state == AUAV_BARO_CMD && type_ == AUAV_BARO)) {
    launchUs_ = time64.Us();
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(cmdBytes_, AUAV_CMD_BYTES)))) spiState_ = poll_state;
    else spiState_ = AUAV_ERROR;
  } else if ((poll_state == AUAV_PITOT_RX && type_ == AUAV_PITOT)
             || (poll_state == AUAV_BARO_RX && type_ == AUAV_BARO)) {
    drdy_ = time64.Us();
    if ((dmaRunning_ = (HAL_OK == spi_.startDma(0xF0, AUAV_READ_BYTES)))) spiState_ = poll_state;
    else spiState_ = AUAV_ERROR;
  }

  return dmaRunning_;
}

void Auav::endDma(void)
{
  if ((spiState_ == AUAV_PITOT_RX && type_ == AUAV_PITOT)
      || (spiState_ == AUAV_BARO_RX && type_ == AUAV_BARO)) {
    uint8_t * inbuf = spi_.endDma();
    // Returns <status> Pressure H,M,L, Temperature H,M L

    // ----- Magic Numbers: constants to be used:  -------------
    // Temp output at 25C:
    // July 2023: (int32_t)((25.0 - -45.0)/(110 - -45.0)) * 0xFFFFFF;
    const int32_t TrefCounts = 7576807;
    // 1% FSO in counts , normalized to % twice
    const double TC50Scale = 100.0 * 100.0 * 167772.2;

    double Pnorm, AP3, BP2, CP, Corr, PCorr;
    double TC50, TCorr, Pnormt, Pnfso, Pcorrt;
    int32_t iCorr, iPraw, iTraw;
    int32_t /*iTCorr,*/ Tdiff;

    // Adjust output for linearization.
    iPraw = (inbuf[1] << 16) + (inbuf[2] << 8) + inbuf[3] - 0x800000;
    iTraw = (inbuf[4] << 16) + (inbuf[5] << 8) + inbuf[6];
    Pnorm = (double) iPraw;
    Pnorm /= (double) 0x7FFFFF; // norm to +-1.0

    AP3 = LIN_A_ * Pnorm * Pnorm * Pnorm;
    BP2 = LIN_B_ * Pnorm * Pnorm;
    CP = LIN_C_ * Pnorm;
    Corr = AP3 + BP2 + CP + LIN_D_;

    PCorr = Pnorm + Corr;                          // norm -1 to +1
    iCorr = (int32_t) (PCorr * (double) 0x7FFFFF); // *= 0.5

    iCorr += 0x800000; // + 0.5 FSO: norm 0 - 16M

    /* iCorr is now in same u24 form as sensor pressure output.
         * The linearity improvements at this stage,
         * without residual TCO/TCS corrections, may be
         * "good enough" for many applications.  */

    // TC50 correction: residual TCO/TCS correction:
    Pnfso = (PCorr + 1.0) / 2.0; // PCorr is +-1.0; Norm to 0 - 1.0

    // Use TC50H above Tref, TC50L below Tref:
    Tdiff = iTraw - TrefCounts; // 24-bit Temp counts

    if (Tdiff > 0) TC50 = TC50H_;
    else TC50 = TC50L_;

    Pnormt = abs(0.5 - Pnfso);

    // The heavy work -- correction = f( P, T ):
    // First, the correction constant TC50 is scaled by distance from offset
    // (zero) pressure (Pnormt, norm to 0.4) and the coefficient Es (norm to +-1.0).
    // This factor is then scaled by the delta from reference temperature to obtain
    // pressure-denominated correction value.
    TCorr = (1.0 - (Es_ * 2.5 * Pnormt)) * TC50 * Tdiff / TC50Scale;
    Pcorrt = Pnfso - TCorr; // Corrected pressure, norm 0 - 1.0

    //	iCorr = (int32_t) (Pcorrt * (double)0xFFFFFF);   // convert back to u24
    //	Return 24-bit values to buffer, in sensor pressure output format.
    //	Application can then will use transfer function to units.
    //	inbuf[1] = (iCorr & 0xFF0000) >> 16;
    //  inbuf[2] = (iCorr & 0xFF00) >> 8;
    //  inbuf[3] = (iCorr & 0xFF);

    PressurePacket p;

    p.drdy = drdy_;
    p.groupDelay = (p.drdy - launchUs_) / 2;
    p.status = (uint16_t) inbuf[0];

    p.temperature = (double) iTraw * 155.0 / 16777216.0 - 45.0 + 273.15;
    p.pressure = off_ + 1.25 * (Pcorrt - osDig_) * fss_;
    p.timestamp = time64.Us();

    rxFifo_.write((uint8_t *) &p, sizeof(p));
  }
  dmaRunning_ = false;
}

bool Auav::display(void)
{
  PressurePacket p;
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name_, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("%10.3f kPa                         |                                        | "
                "%7.1f C |           "
                "   | 0x%04X\n",
                p.pressure / 1000., p.temperature - 273.15, p.status);
    return 1;
  } else {
    misc_printf("%s\n", name_);
  }
  return 0;
}
