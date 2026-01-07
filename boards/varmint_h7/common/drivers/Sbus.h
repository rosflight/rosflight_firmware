/**
 ******************************************************************************
 * File     : Sbus.h
 * Date     : Oct 2, 2023
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

#ifndef SBUS_H_
#define SBUS_H_

#include "DoubleBuffer.h"
#include "CommonConfig.h"

/*
 *
 */
class Sbus : public Status
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
    // UART initializers
    UART_HandleTypeDef * huart, USART_TypeDef * huart_instance, DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud);

  bool poll(void);
  void endDma(void);
  bool startDma(void);
  bool display(void);
  bool lol(void) { return lol_; }

  UART_HandleTypeDef * huart(void) { return huart_; }
  bool isMy(UART_HandleTypeDef * huart) { return huart_ == huart; }

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size)==DoubleBufferStatus::OK; }

private:
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size)==DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  bool lol_;
  uint64_t drdy_;
  uint64_t timeout_;

  uint64_t dtimeout_;
  UART_HandleTypeDef * huart_;
  DMA_HandleTypeDef * hdmaUartRx_;
};

#endif /* SBUS_H_ */
