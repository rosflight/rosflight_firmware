/**
 ******************************************************************************
 * File     : Signal.h
 * Date     : June 6, 2025
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

#ifndef DRIVERS_SIGNAL_H_
#define DRIVERS_SIGNAL_H_

#include "Status.h"

#include <stdio.h>
#include <string.h>

enum class SignalStatus
{
  ERROR=-1,
  EMPTY=0,
  OK=1,
//  OVERWRITE=2,
};

class Signal : public Status
{
  public:

  uint32_t init(uint8_t * packet_buffer, size_t packet_sizeX2 )
  {
    snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Signal");
    initializationStatus_ = DRIVER_OK;

    size_   = packet_sizeX2/2;

    if (packet_buffer==NULL)
    {
      size_ = 0;
      packet_[0] = NULL;
      packet_[1] = NULL;
      initializationStatus_ = DRIVER_NOT_INITIALIZED;
    }

    packet_[0] = packet_buffer;
    packet_[1] = packet_buffer + size_;

    reset();

    return initializationStatus_;
  }

  void reset(void)
  {
    rx_ = 0;
    tx_ = 0;
    memset(packet_[0],0,size_);
    memset(packet_[1],0,size_);
  }

  SignalStatus write(uint8_t * packet, size_t size)
  {
    if ((initializationStatus_ == DRIVER_NOT_INITIALIZED) || (packet==NULL) || (size != size_) ) return SignalStatus::ERROR;
    memcpy(packet_[tx_], packet, size_);
    if(rx_ != tx_) return SignalStatus::OK; //SignalStatus::OVERWRITE; // buffer not empty overwrote next value, still a good write.
    tx_ = !tx_;
    return SignalStatus::OK;
  }

  SignalStatus read(uint8_t * packet, size_t size)
  {
    if ((initializationStatus_ == DRIVER_NOT_INITIALIZED) || (packet==NULL) || (size != size_) ) return SignalStatus::ERROR;
    if(rx_ == tx_) return SignalStatus::EMPTY; // buffer empty
    memcpy(packet, packet_[rx_], size_);
    rx_ = !rx_;
    return SignalStatus::OK;
  }

private:
  volatile uint32_t rx_, tx_;
  size_t size_;
  uint8_t * packet_[2];
};






















#endif /* DRIVERS_SIGNAL_H_ */
