/**
 ******************************************************************************
 * File     : Driver.h
 * Date     : Sep 20, 2023
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

#ifndef DRIVER_H_
#define DRIVER_H_

#include "stm32h7xx_hal.h"

#include "BoardConfig.h"
#include "PacketFifo.h"
#include "Polling.h"
#include "Status.h"

#include <stdint.h>

class Driver : public Status
{
public:
  Driver() { initializationStatus_ = DRIVER_NOT_INITIALIZED; }
  //  bool initGood(void) { return initializationStatus_ == DRIVER_OK; }

  virtual bool display(void) = 0;

  uint16_t rxFifoCount(void) { return rxFifo_.packetCount(); }
  uint16_t rxFifoRead(uint8_t * data, uint16_t size) { return rxFifo_.read(data, size); }
  uint16_t rxFifoReadMostRecent(uint8_t * data, uint16_t size) { return rxFifo_.readMostRecent(data, size); }
  bool drdy(void) { return HAL_GPIO_ReadPin(drdyPort_, drdyPin_); }
  bool dmaRunning(void) { return dmaRunning_; }
  void rotate(double *x) {
    double y[3];
    y[0] = x[0]*rotation_[0] + x[1]*rotation_[1] + x[2]*rotation_[2];
    y[1] = x[0]*rotation_[3] + x[1]*rotation_[4] + x[2]*rotation_[5];
    y[2] = x[0]*rotation_[6] + x[1]*rotation_[7] + x[2]*rotation_[8];
    memcpy(x,y,sizeof(double)*3);
  }

protected:
  PacketFifo rxFifo_;
  GPIO_TypeDef * drdyPort_;
  uint16_t drdyPin_;
  uint16_t sampleRateHz_;
  uint64_t drdy_, timeout_, launchUs_;
  uint64_t groupDelay_ = 0;
  bool dmaRunning_ = 0;
  double rotation_[9];
};

#endif /* DRIVER_H_ */
