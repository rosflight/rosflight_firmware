/**
 ******************************************************************************
 * File     : ImuDriver.h
 * Date     : Jun 12, 2026
 ******************************************************************************
 *
 * Copyright (c) 2026, AeroVironment, Inc.
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

#ifndef IMUDRIVER_H_
#define IMUDRIVER_H_

#include "BoardConfig.h"
#include "DoubleBuffer.h"
#include "Status.h"
#include "misc.h"
#include "Packets.h"
#include "Time64.h"

#define ADIS_OK (0x0000)

extern Time64 time64;

class ImuDriver : public Status, public MiscRotatable
{
public:
  virtual ~ImuDriver() = default;

  virtual void endDma(void) = 0;
  virtual bool startDma(void) = 0;

  bool display(void) {
    ImuPacket p;
    if (read((uint8_t*)&p, sizeof(p))) {
      misc_header(name_, p.header);
      misc_f32(nan(""), nan(""), p.accel[0] / 9.80665, "ax", "%6.2f", "g");
      misc_f32(nan(""), nan(""), p.accel[1] / 9.80665, "ay", "%6.2f", "g");
      misc_f32(nan(""), nan(""), p.accel[2] / 9.80665, "az", "%6.2f", "g");
      double a = sqrt(p.accel[0]*p.accel[0] + p.accel[1]*p.accel[1] + p.accel[2]*p.accel[2]);
      misc_f32(0.8, 1.2, a / 9.80665, "|a|", "%6.2f", "g");

      misc_f32(-1, 1, p.gyro[0] * 57.2958, "p", "%6.2f", "dps");
      misc_f32(-1, 1, p.gyro[1] * 57.2958, "q", "%6.2f", "dps");
      misc_f32(-1, 1, p.gyro[2] * 57.2958, "r", "%6.2f", "dps");
      misc_f32(18, 50, p.temperature - 273.15, "Temp", "%5.1f", "C");
      misc_printf("Count %7.3f s  |", p.dataTime);
      misc_x16(ADIS_OK, p.header.status, "Status");

      misc_printf("\n");
      return true;
    } else {
      misc_printf("%s\n", name_);
    }
    return true;
  }

  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
  bool read(uint8_t* data, uint16_t size) {
    return double_buffer_.read(data, size) == DoubleBufferStatus::OK;
  }

protected:
  bool write(uint8_t* data, uint16_t size) {
    return double_buffer_.write(data, size) == DoubleBufferStatus::OK;
  }

  void finalizePacket(ImuPacket& p) {
    p.header.timestamp = drdy_ - groupDelay_;
    rotate(p.gyro);
    rotate(p.accel);
    p.header.complete = time64.Us();
    write((uint8_t*)&p, sizeof(p));
  }

  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  uint64_t groupDelay_;
  uint16_t drdyPin_;
  uint64_t drdy_;
  uint16_t timeoutMs_;
};

#endif /* IMUDRIVER_H_ */
