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
#include "Auav.h"
#include "Packets.h"
#include "Polling.h"
#include "Time64.h"
#include "misc.h"

extern Time64 time64;

#define AUAV_READ_BYTES 7

DMA_RAM uint8_t auav_dma_txbuf[SPI_DMA_MAX_BUFFER_SIZE];
DMA_RAM uint8_t auav_dma_rxbuf[SPI_DMA_MAX_BUFFER_SIZE];

DTCM_RAM uint8_t pitot_double_buffer[2 * sizeof(PressurePacket)];
DTCM_RAM uint8_t baro_double_buffer[2 * sizeof(PressurePacket)];

#define ROLLOVER 10000

#define STATUS_BARO_START 1
#define STATUS_PITOT_START 2
#define STATUS_PITOT_READ_STATUS 3
#define STATUS_PITOT_READ 4
#define STATUS_BARO_READ 5
#define STATUS_WAITING 0xFF
#define STATUS_IDLE 0

uint32_t Auav::init(uint16_t sample_rate_hz,                                 // Sample rate
                    GPIO_TypeDef * pitot_drdy_port, uint16_t pitot_drdy_pin, // Pitot DRDY
                    GPIO_TypeDef * pitot_cs_port, uint16_t pitot_cs_pin,     // Pitot CS
                    GPIO_TypeDef * baro_drdy_port, uint16_t baro_drdy_pin,   // Baro DRDY
                    GPIO_TypeDef * baro_cs_port, uint16_t baro_cs_pin,       // Baro CS
                    SPI_HandleTypeDef * hspi)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "AuavPitotBaro");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;

  drdyPort_[AUAV_PITOT] = pitot_drdy_port;
  drdyPin_[AUAV_PITOT] = pitot_drdy_pin;
  drdyPort_[AUAV_BARO] = baro_drdy_port;
  drdyPin_[AUAV_BARO] = baro_drdy_pin;

  // These do not run at the same time, so can share the dma buffers.

  spiState_ = STATUS_IDLE;
  dmaRunning_ = false;

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

  // Pitot //////////////////////
  {
    spi_[AUAV_PITOT].init(hspi, auav_dma_txbuf, auav_dma_rxbuf, pitot_cs_port, pitot_cs_pin);

    double_buffer_[AUAV_PITOT].init(pitot_double_buffer, sizeof(pitot_double_buffer));

    char name[] = "Auav (pitot)";
    memset(name_local_[AUAV_PITOT], '\0', sizeof(name_local_[AUAV_PITOT]));
    strcpy(name_local_[AUAV_PITOT], name);
    memset(cmdBytes_[AUAV_PITOT], 0, AUAV_CMD_BYTES);
    cmdBytes_[AUAV_PITOT][0] = 0xAD; // ~100 Hz

    // Vent (zero) pressure is at 0.1 *2^24 nominal output for Gauge Sensor
    osDig_[AUAV_PITOT] = 0.1;  // *2^24;
    fss_[AUAV_PITOT] = 2488.4; // Pa
    off_[AUAV_PITOT] = 0.0;    // Pa
    addr_[AUAV_PITOT] = 43;
    // Status register:
    // 7 0
    // 6 Powered?
    // 5 Busy?
    // 4:3 Mode, 00 = command, 01 = cyclic 10 = idle/sleep
    // 2 Memory Error?
    // 1 Connection Check Fault?
    // 0 Math Saturation (out of range)
    // 0x50 is Powered & Idle/Sleep == Ready.
    sensor_status_ready_[AUAV_PITOT] = 0x50;
  }
  // Baro //////////////////////
  {
    spi_[AUAV_BARO].init(hspi, auav_dma_txbuf, auav_dma_rxbuf, baro_cs_port, baro_cs_pin);

    double_buffer_[AUAV_BARO].init(baro_double_buffer, sizeof(baro_double_buffer));

    char name[] = "Auav (baro) ";
    memset(name_local_[AUAV_BARO], '\0', sizeof(name_local_[AUAV_BARO]));
    strcpy(name_local_[AUAV_BARO], name);
    memset(cmdBytes_[AUAV_BARO], 0, AUAV_CMD_BYTES);
    cmdBytes_[AUAV_BARO][0] = 0xAC; // ~100 Hz

    // Vent (zero) pressure is at 0.1 *2^24 nominal output for Gauge Sensor
    osDig_[AUAV_BARO] = 0.1;    // *2^24;
    fss_[AUAV_BARO] = 100000.0; // Pa
    off_[AUAV_BARO] = 25000.0;  // Pa
    addr_[AUAV_BARO] = 47;
    // Status register:
    // 7 always 0
    // 6 Powered?
    // 5 Busy?
    // 4:3 Mode, 00 = Normal Operation
    // 2 Memory Error?
    // 1 Sensor Configuration, always 0
    // 0 ALU error (out of range)
    // 0x40 is Powered & Normal Operation == Ready
    sensor_status_ready_[AUAV_BARO] = 0x40;
  }

  // Force AUAV into SPI Mode
  HAL_GPIO_WritePin(baro_cs_port, baro_cs_pin, GPIO_PIN_SET);   // set high (should be there already)
  HAL_GPIO_WritePin(pitot_cs_port, pitot_cs_pin, GPIO_PIN_SET); // set high (should be there already)
  time64.dUs(100);

  //	// "Provide a 5-10 us low pulse
  //	HAL_GPIO_WritePin(pitot_cs_port, pitot_cs_pin, GPIO_PIN_RESET); // low for 20 microseconds
  //	time64.dUs(20); // 5 to 20 us in data sheet.
  //	HAL_GPIO_WritePin(pitot_cs_port, pitot_cs_pin, GPIO_PIN_SET); // set high (should be there already)
  //	// "Delay 5 us
  //	time64.dUs(5);
  //	// "Provide a 5-10 us low pulse
  //	HAL_GPIO_WritePin(baro_cs_port, baro_cs_pin, GPIO_PIN_RESET); // low for 20 microseconds
  //	time64.dUs(20); // 5 to 20 us in data sheet.
  //	HAL_GPIO_WritePin(baro_cs_port, baro_cs_pin, GPIO_PIN_SET); // set high (should be there already)
  //	// "Delay 5 us
  //	time64.dUs(25); // > 5

  // Force SPI Mode.
  for (int i = 0; i < 2; i++) {
    uint8_t tx[3] = {0xF0, 0, 0};
    uint8_t junk[3] = {0, 0, 0};
    spi_[i].rx(tx, junk, 3, 100);
  }

  for (int i = 0; i < 2; i++) {
    groupDelay_[i] = 0;

    // Read Status
    uint8_t tx = 0xF0;
    uint8_t sensor_status = 0x00;
    HAL_StatusTypeDef hal_status = spi_[i].rx(&tx, &sensor_status, 1, 200);
    misc_printf("HAL Status = 0x%04X : ", hal_status);
    misc_printf("%s Status = 0x%02X (0x%02X) - ", name_local_[i], sensor_status, sensor_status_ready_[i]);

    if (sensor_status == sensor_status_ready_[i]) {
      misc_printf("OK\n");
    } else {
      misc_printf("ERROR\n");
      if(i==AUAV_PITOT) initializationStatus_ |= AUAV_PITOT_ERROR; //DRIVER_SELF_DIAG_ERROR;
      else initializationStatus_ |= AUAV_BARO_ERROR; //DRIVER_SELF_DIAG_ERROR;
    }
  }
  for (int i = 0; i < 2; i++) {
    // Calibration constants
    int32_t i32A = 0, i32B = 0, i32C = 0, i32D = 0, i32TC50HLE = 0;
    int8_t i8TC50H = 0, i8TC50L = 0, i8Es = 0;

    // These i32 Reads return 2 register values merged as int32
    // i32 then normalized to +/- 1.0
    // Note that Diff data block is shifted 4 down from ABS locations
    i32A = readCfg(addr_[i], &spi_[i]);
    i32B = readCfg(addr_[i] + 2, &spi_[i]);
    i32C = readCfg(addr_[i] + 4, &spi_[i]);
    i32D = readCfg(addr_[i] + 6, &spi_[i]);
    i32TC50HLE = readCfg(addr_[i] + 8, &spi_[i]);

    LIN_A_[i] = (double) (i32A) / (double) (0x7FFFFFFF);
    LIN_B_[i] = (double) (i32B) / (double) (0x7FFFFFFF);
    LIN_C_[i] = (double) (i32C) / (double) (0x7FFFFFFF);
    LIN_D_[i] = (double) (i32D) / (double) (0x7FFFFFFF);

    i8TC50H = (i32TC50HLE >> 24) & 0xFF;
    i8TC50L = (i32TC50HLE >> 16) & 0xFF;
    i8Es = (i32TC50HLE) &0xFF;

    Es_[i] = (double) (i8Es) / (double) (0x7F);       // norm to +/- 1.0
    TC50H_[i] = (double) (i8TC50H) / (double) (0x7F); // norm to +/- 1.0
    TC50L_[i] = (double) (i8TC50L) / (double) (0x7F); // norm to +/- 1.0
  }
  misc_printf("\n");

  return initializationStatus_;
}

int32_t Auav::readCfg(uint8_t address, Spi * spi)
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

  int32_t value = (hi[1] << 24) | (hi[2] << 16) | (lo[1] << 8) | lo[2];

  return value;
}

bool Auav::poll(uint64_t poll_counter)
{
  PollingState poll_state = (PollingState) (poll_counter % (ROLLOVER / POLLING_PERIOD_US));

  if (poll_state == 0) // Start Baro Read
  {
    spiState_ = STATUS_IDLE;
    if ((dmaRunning_ = (HAL_OK == spi_[AUAV_BARO].startDma(cmdBytes_[AUAV_BARO], AUAV_CMD_BYTES)))) {
      spiState_ = STATUS_BARO_START;
    }
  }
  return false;
}

void Auav::endDma(void)
{
  if (spiState_ == STATUS_BARO_START) { // Done starting Baro, Start Pitot
    spi_[AUAV_BARO].endDma();           // close chip select, data ignored
    spiState_ = STATUS_IDLE;
    if ((dmaRunning_ = (HAL_OK == spi_[AUAV_PITOT].startDma(cmdBytes_[AUAV_PITOT], AUAV_CMD_BYTES)))) {
      spiState_ = STATUS_PITOT_START;
    }

  } else if (spiState_ == STATUS_PITOT_START) { // Done starting Pitot, Wait for Pitot Read
    spi_[AUAV_PITOT].endDma();                  // close chip select, data ignored
    spiState_ = STATUS_WAITING;
  } else if (spiState_ == STATUS_PITOT_READ_STATUS) { // Done reading Pitot
    spi_[AUAV_PITOT].endDma();                        // close chip select, data ignored
    spiState_ = STATUS_IDLE;
    if ((dmaRunning_ = (HAL_OK == spi_[AUAV_PITOT].startDma(0xF0, AUAV_READ_BYTES)))) { spiState_ = STATUS_PITOT_READ; }
  } else if (spiState_ == STATUS_PITOT_READ) {   // Done reading Pitot
    uint8_t * inbuf = spi_[AUAV_PITOT].endDma(); // close chip select, data ignored
    PressurePacket p;
    makePacket(&p, inbuf, AUAV_PITOT);
    spiState_ = STATUS_IDLE;
    if(p.header.status==sensor_status_ready_[AUAV_PITOT]) // PTT uncomment if needed
    {
      p.read_complete = time64.Us();
      write2((uint8_t *) &p, sizeof(p), AUAV_PITOT);
    }
  } else if (spiState_ == STATUS_BARO_READ) {   // Done starting Baro
    uint8_t * inbuf = spi_[AUAV_BARO].endDma(); // close chip select, data ignored
    PressurePacket p;
    makePacket(&p, inbuf, AUAV_BARO);
    spiState_ = STATUS_IDLE;
    if(p.header.status==sensor_status_ready_[AUAV_BARO]) // PTT uncomment this when we fix the sensor.
    {
      p.read_complete = time64.Us();
      write2((uint8_t *) &p, sizeof(p), AUAV_BARO);
   }
  } else {
    spiState_ = STATUS_IDLE;
  }
  dmaRunning_ = false;
}

void Auav::drdyIsr(uint64_t timestamp, uint16_t exti_pin)
{
  if (exti_pin == drdyPin_[AUAV_PITOT]) // Start Pitot Read
  {
    drdy_[AUAV_PITOT] = time64.Us();
    spiState_ = STATUS_IDLE;
    time64.dUs(20);
    if ((dmaRunning_ =
           (HAL_OK
            == spi_[AUAV_PITOT].startDma(0xF0, 1)))) // Read Status, why do I need to do this to make the read work?

    {
      spiState_ = STATUS_PITOT_READ_STATUS;
    }

  } else if (exti_pin == drdyPin_[AUAV_BARO]) { // Start Baro Read
    drdy_[AUAV_BARO] = time64.Us();
    spiState_ = STATUS_IDLE;
    if ((dmaRunning_ = (HAL_OK == spi_[AUAV_BARO].startDma(0xF0, AUAV_READ_BYTES)))) { spiState_ = STATUS_BARO_READ; }
  }
  // else not us.
}

void Auav::makePacket(PressurePacket * p, uint8_t * inbuf, uint8_t device)
{

  int32_t iPraw = ((inbuf[1] << 16) | (inbuf[2] << 8) | inbuf[3]) - 0x800000;
  int32_t iTemp = ((inbuf[4] << 16) | (inbuf[5] << 8) | inbuf[6]);

  double Pnorm = (double) iPraw;
  Pnorm /= (double) 0x7FFFFF; // norm to +-1.0

  //  double AP3 = LIN_A_[device] * Pnorm * Pnorm * Pnorm;
  //  double BP2 = LIN_B_[device] * Pnorm * Pnorm;
  //  double CP  = LIN_C_[device] * Pnorm;
  //  double Corr = AP3 + BP2 + CP + LIN_D_[device];
  //  double PCorr = Pnorm + Corr; // norm -1 to +1

  // PTT Somewhat more efficient way to Compute Corr
  double Pcorr = Pnorm + ((LIN_A_[device] * Pnorm + LIN_B_[device]) * Pnorm + LIN_C_[device]) * Pnorm + LIN_D_[device];

#if 0 // Basic uncorrected values
  int32_t iPcorr = (int32_t) (Pcorr * (double) 0x7FFFFF); // *= 0.5
  //iPcorr += 0x800000; // + 0.5 FSO: norm 0 - 16M
  uint32_t Pdig = iPcorr + 0x800000; // *= 0.5
#else // additional temperature adjustment
  // ----- Magic Numbers: constants to be used:  -------------
  // Temp output at 25C:
  // July 2023: (int32_t)((25.0 - -45.0)/(110 - -45.0)) * 0xFFFFFF;
  const int32_t TrefCounts = 7576807;
  const double TC50Scale = 100.0 * 100.0 * 167772.2; // 1% FSO in counts , normalized to % twice

  // TC50 correction: residual TCO/TCS correction:
  double Pnfso = (Pcorr + 1.0) / 2.0; // PCorr is +-1.0; Norm to 0 - 1.0

  // Use TC50H above Tref, TC50L below Tref:
  int32_t Tdiff = iTemp - TrefCounts; // 24-bit Temp counts

  double TC50 = 0;
  if (Tdiff > 0) TC50 = TC50H_[device];
  else TC50 = TC50L_[device];

  double Pdiff = abs(Pnfso - 0.5);

  // The heavy work -- correction = f( P, T ):
  // First, the correction constant TC50 is scaled by distance from offset
  // (zero) pressure (Pnormt, norm to 0.4) and the coefficient Es (norm to +-1.0).
  // This factor is then scaled by the delta from reference temperature to obtain
  // pressure-denominated correction value.
  double Tcorr = (1.0 - (Es_[device] * 2.5 * Pdiff)) * Tdiff * TC50 / TC50Scale;
  double PCorrt = Pnfso - Tcorr; // Corrected pressure, norm 0 - 1.0
                                 //  uint32_t Pcomp = (uint32_t)(PCorrt*(double)0xFFFFFF);
  uint32_t Pdig = (uint32_t) (PCorrt * (double) 0xFFFFFF);
#endif

  p->header.status = (uint16_t) inbuf[0];

  p->temperature = (double) iTemp * 155.0 / 16777216.0 - 45.0 + 273.15;
  p->pressure = off_[device] + 1.25 * ((double) Pdig / 16777216.0 - osDig_[device]) * fss_[device];
  p->header.timestamp = drdy_[device];
}

bool Auav::display(void)
{
  PressurePacket p;

  if (read2((uint8_t *) &p, sizeof(p), AUAV_BARO)) {
    misc_header(name_local_[AUAV_BARO], p.header.timestamp, p.read_complete);

    misc_f32(99, 101, p.pressure / 1000., "Press", "%6.2f", "kPa");
    misc_f32(18, 50, p.temperature - 273.15, "Temp", "%5.1f", "C");
    misc_x16(sensorOk(AUAV_BARO), p.header.status, "Status");
    misc_printf("\n");
    //return 1;
  } else {
    misc_printf("%s\n", name_local_[AUAV_BARO]);
  }

  if (read2((uint8_t *) &p, sizeof(p), AUAV_PITOT)) {
    misc_header(name_local_[AUAV_PITOT], p.header.timestamp, p.read_complete);
    misc_f32(-5.0, 5.0, p.pressure, "Press", "%6.2f", "Pa");
    misc_f32(18, 50, p.temperature - 273.15, "Temp", "%5.1f", "C");
    misc_x16(sensorOk(AUAV_PITOT), p.header.status, "Status");
    misc_printf("\n");
    //return 1;
  } else {
    misc_printf("%s\n", name_local_[AUAV_PITOT]);
  }

  return 0;
}
