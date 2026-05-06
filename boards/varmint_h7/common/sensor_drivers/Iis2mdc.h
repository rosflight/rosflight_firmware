/**
 ******************************************************************************
 * File     : Iis2mdc.h
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

#ifndef IIS2MDC_H_
#define IIS2MDC_H_

#include "DoubleBuffer.h"
#include "BoardConfig.h"

#include "Spi.h"
#include "misc.h"
#include "Polling.h"

#define IIS2MDC_OK (0x0F)

/*
 *
 */
class Iis2mdc : public Status, public MiscRotatable
{
  /**
     * \brief
     *
     *
     */
public:
  uint32_t init(
    // Driver initializers
    uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
    uint16_t drdy_pin,                                 // Reset GPIO Pin
    // SPI initializers
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Chip Select GPIO Port
    uint16_t cs_pin,                                   // Chip Select GPIO Pin
    const double *rotation
  );
  // bool poll(void);
  bool poll(uint64_t poll_counter);
  void endDma(void);

  bool display(void);

  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
  bool isMy(SPI_HandleTypeDef * hspi) { return hspi == spi_.hspi(); }
  SPI_HandleTypeDef * hspi(void) { return spi_.hspi(); }

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size)==DoubleBufferStatus::OK; }

private:
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size)==DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  // SPI Stuff
  Spi spi_;
  PollingState spiState_;
  uint16_t drdyPin_;
  uint64_t drdy_;
  bool dmaRunning_;
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t readRegister(uint8_t address);

};

#endif /* IIS2MDC_H_ */
