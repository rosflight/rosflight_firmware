/**
 ******************************************************************************
 * File     : Liv4f.h
 * Date     : Jun 18, 2025
 ******************************************************************************
 *
 * Copyright (c) 2025, AeroVironment, Inc.
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

#ifndef DRIVERS_LIV4F_H_
#define DRIVERS_LIV4F_H_

#include "DoubleBuffer.h"
#include "BoardConfig.h"
#include "GpsDriver.h"
#include "Packets.h"
#include "Status.h"

#define LIV4F_MAX_PAYLOAD_BYTES (128)

typedef struct //__attribute__((__packed__))
{
  uint16_t length;
  uint16_t n_star;
  uint8_t checksum;
  uint8_t payload[LIV4F_MAX_PAYLOAD_BYTES];
} Liv4fFrame;

class Liv4f : public Status, public GpsDriver
{
public:
  uint32_t init(
    // Driver initializers
    uint16_t sample_rate_hz, GPIO_TypeDef * pps_port, uint16_t pps_pin,
    // UART initializers
    UART_HandleTypeDef * huart, USART_TypeDef * huart_instance,
    DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud_desired);

  // GpsDriver interface implementation
  void endDma(void) override;
  void pps(uint64_t pps_timestamp) override;
  bool startDma(void) override;

  bool display(void) override { return displayGnss(name_); }
  bool parseByte(uint8_t c, Liv4fFrame * p);

private:
  // Liv4f-specific member variables
  GnssPacket gnss_;
  bool hasPps_;
  uint64_t gotPvraw_;
  uint64_t gotZda_;
  uint64_t gotKfcov_;
  uint64_t pps_;

  char calculateNMEAChecksum(char *sentence);
  bool lookFor(const char* match, char* buffer, int num_received);

};

#endif /* DRIVERS_LIV4F_H_ */
