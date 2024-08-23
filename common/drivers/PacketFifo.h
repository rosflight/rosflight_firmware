/**
 ******************************************************************************
 * File     : PacketFifo.h
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

#ifndef PACKETFIFO_H_
#define PACKETFIFO_H_

#include <stdio.h>
#include <string.h>

#define PACKET_FIFO_MAX_BUFFERS 128

typedef struct
{
  uint16_t size;
  uint8_t * data;
} Packet;

class PacketFifo
{
public:
  PacketFifo() { initializationStatus_ = DRIVER_NOT_INITIALIZED;}
  bool initGood(void) { return initializationStatus_== DRIVER_OK;}

  void init(uint16_t max_packets, uint16_t max_data_size, uint8_t * buffer_head)
  {
    initializationStatus_= DRIVER_OK;
    if(buffer_head==NULL) // Maybe check for a valid range?
    {
      packetCountMax_=0;
      dataSizeMax_ = 0;
      initializationStatus_= DRIVER_NOT_INITIALIZED;
    }
    else
    {
      if (max_packets > PACKET_FIFO_MAX_BUFFERS) packetCountMax_ = PACKET_FIFO_MAX_BUFFERS;
      else packetCountMax_ = max_packets;
      dataSizeMax_ = max_data_size;
      for (uint16_t i = 0; i < packetCountMax_; i++) {
        packet_[i].size = dataSizeMax_;
        packet_[i].data = buffer_head + i * dataSizeMax_;
      }
    }
    reset();
  }

  void reset(void)
  {
    head_ = 0;
    tail_ = 0;
  }

  uint16_t write(uint8_t * data, uint16_t size)
  {
    if (packetCountMax_ == 0) return false;
    if (tail_ == (head_ + 1) % packetCountMax_) return false;
    size = (size > dataSizeMax_) ? dataSizeMax_ : size;
    packet_[head_].size = size;
    memcpy(packet_[head_].data, data, size);
    if (++head_ == packetCountMax_) head_ = 0;
    return size;
  }
  /**
     * @fn uint16_t read(uint8_t*, uint16_t)
     * @brief Gets oldest buffer in the Fifo.
     *
     * @param data Data gets copied here
     * @param size Maximum size of data buffer
     * @return Actual size of data buffer read
     */
  uint16_t read(uint8_t * data, uint16_t size)
  {
    if (head_ == tail_) return 0; // buffer is empty
    if (size > packet_[tail_].size) size = packet_[tail_].size;
    memcpy(data, packet_[tail_].data, size);
    if (++tail_ == packetCountMax_) tail_ = 0;
    return size;
  }
  /**
     * @fn uint16_t readMostRecent(uint8_t*, uint16_t)
     * @brief Get the most recent data in the Fifo, drop the older buffers.
     *
     * @param data Data gets copied here
     * @param size Maximum size of data buffer
     * @return Actual size of data buffer read
     */
  uint16_t readMostRecent(uint8_t * data, uint16_t size)
  {
    if (head_ == tail_) return 0; // buffer is empty
    if (head_ == 0) tail_ = packetCountMax_ - 1;
    else tail_ = head_ - 1;
    return read(data, size);
  }

  /**
     * @fn Packet peek*(void)
     * @brief	Get a pointer to the current read buffer.
     *        User must check packetCount()>0 before calling this to ensure data is not volatile.
     *
     * @return Pointer to the packet with index dataOut_.
     *
     */
  Packet * peek(void) { return &(packet_[tail_]); }

  uint16_t packetCount(void)
  {
    if (packetCountMax_ == 0) return 0;
    return (packetCountMax_ + head_ - tail_) % packetCountMax_;
  }
  uint16_t packetCountMax(void) { return packetCountMax_; }

private:
  volatile uint32_t head_, tail_;
  uint32_t dataSizeMax_;
  uint32_t packetCountMax_ = 0;

  volatile uint32_t bufferSize_;
  Packet packet_[PACKET_FIFO_MAX_BUFFERS];
  uint32_t initializationStatus_ = DRIVER_NOT_INITIALIZED;
};

#endif /* PACKETFIFO_H_ */
