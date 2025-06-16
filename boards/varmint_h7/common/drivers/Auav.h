/**
 ******************************************************************************
 * File     : Auav.h
 * Date     : Mar 25, 2023
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

// Part number is AUAV-L10G-M25125-RR-N

#ifndef AUAV_H_
#define AUAV_H_

#include <DoubleBuffer.h>
#include "BoardConfig.h"
#include "Packets.h"
#include "Spi.h"

#define AUAV_PITOT 0
#define AUAV_BARO 1

#define AUAV_CMD_BYTES 3

class Auav : public Status
{
  /**
     * \brief
     *
     *
     */
public:
  uint32_t init(uint16_t sample_rate_hz,                                 // Sample rate
                GPIO_TypeDef * pitot_drdy_port, uint16_t pitot_drdy_pin, // Pitot DRDY
                GPIO_TypeDef * pitot_cs_port, uint16_t pitot_cs_pin,     // Pitot CS
                GPIO_TypeDef * baro_drdy_port, uint16_t baro_drdy_pin,   // Baro DRDY
                GPIO_TypeDef * baro_cs_port, uint16_t baro_cs_pin,       // Baro CS
                SPI_HandleTypeDef * hspi);
  bool poll(uint64_t poll_counter);
  void endDma(void);
  bool display(void);

  bool startTxDma(void);
  void endTxDma(void);

  bool isMy(uint16_t exti_pin) { return (drdyPin_[0] == exti_pin) || (drdyPin_[1] == exti_pin); }
  bool isMy(SPI_HandleTypeDef * hspi) { return (hspi == spi_[0].hspi()); }
  SPI_HandleTypeDef * hspi(void) { return spi_[0].hspi(); }

  void drdyIsr(uint64_t timestamp, uint16_t exti_pin);

  bool read2(uint8_t * data, uint16_t size, uint8_t id) { return double_buffer_[id].read(data, size)==DoubleBufferStatus::OK; }
  bool write2(uint8_t * data, uint16_t size, uint8_t id) { return double_buffer_[id].write(data, size)==DoubleBufferStatus::OK; }

  uint8_t sensorOk(uint8_t id) { return sensor_status_ready_[id]; }
  bool read(uint8_t * data, uint16_t size) { return read2(data,size,AUAV_PITOT); }

private:
  bool write(uint8_t * data, uint16_t size) { return write2(data, size,AUAV_PITOT); }

  void makePacket(PressurePacket * p, uint8_t * inbuff, uint8_t device);
  int32_t readCfg(uint8_t address, Spi * spi);
  // SPI Stuff
  Spi spi_[2];
  uint8_t spiState_;
  uint8_t cmdBytes_[2][AUAV_CMD_BYTES];
  uint8_t addr_[2];

  double LIN_A_[2], LIN_B_[2], LIN_C_[2], LIN_D_[2], Es_[2], TC50H_[2], TC50L_[2];
  double osDig_[2], fss_[2], off_[2];
  uint8_t sensor_status_ready_[2];
  char name_local_[2][16]; // for display

  DoubleBuffer double_buffer_[2];
  GPIO_TypeDef * drdyPort_[2];
  uint16_t drdyPin_[2];
  uint16_t sampleRateHz_;

  uint64_t drdy_[2], timeout_[2];
  uint64_t groupDelay_[2];
  bool dmaRunning_;
};

#endif /* AUAV_H_ */
