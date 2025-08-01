/**
 ******************************************************************************
 * File     : DLHRL20G.h
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

#ifndef DLHRL20G_H_
#define DLHRL20G_H_

#include "DoubleBuffer.h"
#include "BoardConfig.h"

#include "Packets.h"
#include "Time64.h"

#define DLHRL20G_OK (0x40)

#define DLHRL20G_I2C_ADDRESS (0x29)
/*
 *
 */
class DlhrL20G : public Status
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
    I2C_HandleTypeDef * hi2c, uint16_t i2c_address     // I2C initializers
  );
  bool poll(uint64_t poll_offset);
  void endDma(void);
  bool display(void);
  // I2C_HandleTypeDef* hi2c(void) {return hi2c_;}
  bool isMy(I2C_HandleTypeDef * hi2c) { return hi2c_ == hi2c; }

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size)==DoubleBufferStatus::OK; }

private:
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size)==DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  I2C_HandleTypeDef * hi2c_;
  uint16_t address_;
  uint8_t cmdByte_;
  double dtMs_;
  GPIO_TypeDef * drdyPort_;
  uint16_t drdyPin_;
  uint64_t drdy_;
  uint16_t sampleRateHz_;
};

#endif /* DLHRL20G_H_ */
