/**
 ******************************************************************************
 * File     : ByteFifo.h
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

#ifndef BYTEFIFO_H_
#define BYTEFIFO_H_

#include <stdio.h>

#include <Status.h>

class ByteFifo : public Status
{
public:
  bool initGood(void) { return initializationStatus_ == DRIVER_OK; }

  void init(uint16_t buffer_size, uint8_t * buffer)
  {
    snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "ByteFifo");
    if (buffer == NULL) {
      bufferSize_ = 0;
      buffer_ = buffer;
      bufferEnd_ = buffer_;
    } else {
      bufferSize_ = buffer_size;
      buffer_ = buffer;
      bufferEnd_ = buffer_ + bufferSize_ - 1;
    }
    flush();
  }

  void flush(void)
  {
    head_ = buffer_;
    tail_ = buffer_;
  }

  bool write(uint8_t data)
  {
    if (bufferSize_ == 0) return false;                                 // buffer does not exist
    if ((bufferSize_ + tail_ - head_) % bufferSize_ == 1) return false; // buffer full
    *(head_++) = data;

    if (head_ > bufferEnd_) head_ = buffer_;

    return true;
  }
  bool writeBlock(uint8_t * data, uint32_t size)
  {
    if (bufferSize_ == 0) return false; // buffer does not exist
    uint32_t i;
    for (i = 0; i < size; i++) {
      if (!write(data[i])) break;
    }
    return i;
  }

  bool read(uint8_t * data)
  {
    if (head_ == tail_) return false; // buffer is empty
    *data = *(tail_++);
    if (tail_ > bufferEnd_) tail_ = buffer_;
    return true;
  }

  uint16_t byteCount(void)
  {
    if (bufferSize_ == 0) return 0; // buffer does not exist
    return (bufferSize_ + head_ - tail_) % bufferSize_;
  }

private:
  volatile uint8_t *head_, *tail_;
  volatile uint8_t *buffer_, *bufferEnd_;
  uint32_t bufferSize_ = 0;
};

#endif /* BYTEFIFO_H_ */
