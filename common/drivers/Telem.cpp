/**
 ******************************************************************************
 * File     : Telem.cpp
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

#include <Telem.h>

#include <Time64.h>
#include <misc.h>

#include <signal.h>

#include <Callbacks.h>
#include <Packets.h>

extern Time64 time64;

#define TELEM_RX_BUFFER_SIZE (4096) // Use a multiple of 32!

DTCM_RAM uint8_t telem_fifo_rx_buffer[TELEM_RX_BUFFER_SIZE];

#define TELEM_DMA_TX_BUFFER_SIZE (SERIAL_MAX_PAYLOAD_SIZE)
DMA_RAM uint8_t telem_dma_txbuf[TELEM_DMA_TX_BUFFER_SIZE];

#define SERIAL_TX_FIFO_BUFFERS0 (4)
#define SERIAL_TX_FIFO_BUFFERS1 (10)
#define SERIAL_TX_FIFO_BUFFERS2 (SERIAL_TX_FIFO_BUFFERS)

DTCM_RAM uint8_t telem_fifo_tx_buffer0[SERIAL_TX_FIFO_BUFFERS0 * sizeof(SerialTxPacket)];
DTCM_RAM uint8_t telem_fifo_tx_buffer1[SERIAL_TX_FIFO_BUFFERS1 * sizeof(SerialTxPacket)];
DTCM_RAM uint8_t telem_fifo_tx_buffer2[SERIAL_TX_FIFO_BUFFERS2 * sizeof(SerialTxPacket)];
DTCM_RAM PacketFifo telem_tx_fifos[SERIAL_QOS_FIFOS];

DATA_RAM uint8_t * telem_fifo_tx_buffer[SERIAL_QOS_FIFOS] = {telem_fifo_tx_buffer0, telem_fifo_tx_buffer1,
                                                             telem_fifo_tx_buffer2};

uint32_t Telem::init(
  // Driver initializers
  uint16_t sample_rate_hz,
  // UART initializers
  UART_HandleTypeDef * huart, USART_TypeDef * huart_instance, DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud
  //,void (*RxISR) (struct __UART_HandleTypeDef *huart)
)
{
  // Common initializations
  sampleRateHz_ = sample_rate_hz;
  txFrameSizeUs_ = (uint64_t) 1000000 / sampleRateHz_ * 95 / 100; // one period - 5% for margin.
  txFrameEndUs_ = 0 + txFrameSizeUs_;
  usPerByte_ = (uint64_t) 1000000 * 10 / baud; // assume 10 bits/byte.
  txFifo_ = telem_tx_fifos;
  uint32_t serial_tx_fifo_buffer_size[SERIAL_QOS_FIFOS] = {SERIAL_TX_FIFO_BUFFERS0, SERIAL_TX_FIFO_BUFFERS1,
                                                           SERIAL_TX_FIFO_BUFFERS2};

  for (int n = 0; n < SERIAL_QOS_FIFOS; n++)
    txFifo_[n].init(serial_tx_fifo_buffer_size[n], sizeof(SerialTxPacket),
                    telem_fifo_tx_buffer[n]);               // Packet Fifo
  rxFifo_.init(TELEM_RX_BUFFER_SIZE, telem_fifo_rx_buffer); // byte Fifo

  txIdle_ = true;

  // Telem-specific

  huart_ = huart;
  hdmaUartRx_ = hdma_uart_rx;

  huart_->Instance = huart_instance;
  huart_->Init.BaudRate = baud;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.StopBits = UART_STOPBITS_1;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  huart_->Init.HwFlowCtl = UART_HWCONTROL_NONE; //UART_HWCONTROL_RTS_CTS;
  huart_->Init.OverSampling = UART_OVERSAMPLING_16;
  huart_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart_->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(huart_) != HAL_OK) return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_SetTxFifoThreshold(huart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_SetRxFifoThreshold(huart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_DisableFifoMode(huart_) != HAL_OK) return DRIVER_HAL_ERROR;

  return DRIVER_OK;
}

uint32_t Telem::reset_baud(uint32_t baud)
{
  if (baud != huart_->Init.BaudRate) {
    huart_->Init.BaudRate = baud;
    if (HAL_UART_Init(huart_) != HAL_OK) return DRIVER_HAL_ERROR;
  }
  return DRIVER_OK;
}

void Telem::poll(void)
{
  // TX
  if (txIdle_) { txStart(); }
}

bool Telem::rxStart(void) // RX DMA
{
  // Enable interrupt
  ATOMIC_SET_BIT(huart_->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);

  return true;
}

void Telem::rxIsrCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance->ISR & UART_FLAG_RXNE) {
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
    rxFifo_.write(huart->Instance->RDR);
  }
}

uint16_t Telem::writePacket(SerialTxPacket * p)
{
  p->timestamp = time64.Us();
  p->packetSize = sizeof(SerialTxPacket) + p->payloadSize - SERIAL_MAX_PAYLOAD_SIZE;
  if (p->qos < 0x02) return txFifo_[0].write((uint8_t *) p, p->packetSize);
  else if (p->qos < 0xFF) return txFifo_[1].write((uint8_t *) p, p->packetSize);
  else return txFifo_[2].write((uint8_t *) p, p->packetSize);
}

bool Telem::newPacket(SerialTxPacket * p)
{
  uint16_t size = 0;
  int64_t time_now_us = (int64_t) time64.Us();
  int64_t time_remaining_us = txFrameEndUs_ - time_now_us;

  if (txFifo_[0].packetCount() > 0) {
    size = txFifo_[0].read((uint8_t *) p, sizeof(SerialTxPacket));
    if (p->qos == 0x00) { txFrameEndUs_ = time_now_us + txFrameSizeUs_; }
  } else if ((txFifo_[1].packetCount() > 0) && ((int64_t) usPerByte_ * txFifo_[1].peek()->size < time_remaining_us)) {
    size = txFifo_[1].read((uint8_t *) p, sizeof(SerialTxPacket));
  } else if ((txFifo_[2].packetCount() > 0) && ((int64_t) usPerByte_ * txFifo_[2].peek()->size < time_remaining_us)) {
    size = txFifo_[2].read((uint8_t *) p, sizeof(SerialTxPacket));
  }

  if ((size != 0) && (size == p->packetSize)) {
    return true;
  } else {
    return false;
  }
}

bool Telem::txStart(void) // Transmit complete callback.
{
  txIdle_ = false;
  SerialTxPacket p = {0};

  if (newPacket(&p)) {

    memcpy(telem_dma_txbuf, p.payload, p.payloadSize);

    if (HAL_UART_Transmit_DMA(huart_, telem_dma_txbuf, p.payloadSize) != HAL_OK) txIdle_ = true;
  } else {
    txIdle_ = true;
  }
  return !txIdle_;
}
