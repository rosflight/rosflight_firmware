/**
 ******************************************************************************
 * File     : MS4525.h
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

#ifndef MS4525_H_
#define MS4525_H_

#include "DoubleBuffer.h"
#include "BoardConfig.h"
#include "Packets.h"
#include "Time64.h"
#include  "Polling.h"

#define MS4525_I2C_ADDRESS (0x28)
/*
 *
 */
class Ms4525 : public Status
{
  /**
     * \brief
     *
     *
     */
public:
  uint32_t init(
    // Driver initializers
    uint16_t sample_rate_hz,
    // I2C initializers
    I2C_HandleTypeDef * hi2c, // The SPI handle
    uint16_t i2c_address      // Chip select Port
  );
  bool poll(uint64_t poll_counter);

  void endDma(void);
  bool display(void);
  // I2C_HandleTypeDef* hi2c(void) {return hi2c_;}
  bool isMy(I2C_HandleTypeDef * hi2c) { return hi2c_ == hi2c; }

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size)==DoubleBufferStatus::OK; }

private:
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size)==DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  I2C_HandleTypeDef * hi2c_;
  PollingState i2cState_;
  uint16_t address_;
  double dtMs_;
  uint64_t drdy_;
  bool write_data_;
};

#endif /* DLHRL20G_H_ */
