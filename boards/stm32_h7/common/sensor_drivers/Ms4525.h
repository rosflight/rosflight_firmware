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

#include "Async.h"
#include "BoardConfig.h"
#include "DoubleBuffer.h"
#include "I2cBus.h"
#include "Packets.h"
#include "Polling.h"
#include "Time64.h"

#define MS4525_I2C_ADDRESS (0x28)

class STM32H7Board;

class Ms4525 : public Status
{
public:
  uint32_t init(uint16_t sample_rate_hz, I2C_HandleTypeDef * hi2c, uint16_t i2c_address);

  void attach_bus(I2cBus & bus) { async_bus_ = &bus; }

  bool poll(uint64_t poll_counter);
  void start(STM32H7Board & board, int32_t poll_phase_offset = 0);
  bool display(void);

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size) == DoubleBufferStatus::OK; }

private:
  AsyncTask<void> run();
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size) == DoubleBufferStatus::OK; }

  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  uint16_t address_;
  uint64_t drdy_;
  I2cBus * async_bus_ = nullptr;
  AcquisitionSignal poll_signal_;
  AsyncTask<void> task_;
};

#endif /* DLHRL20G_H_ */



