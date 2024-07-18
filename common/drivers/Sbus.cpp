/**
 ******************************************************************************
 * File     : Sbus.cpp
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

#include <Packets.h>
#include <Sbus.h>
#include <Time64.h>
#include <misc.h>

extern Time64 time64;

typedef struct __attribute__((__packed__))
{
  uint8_t header; // 0x0F
  unsigned int chan0 : 11;
  unsigned int chan1 : 11;
  unsigned int chan2 : 11;
  unsigned int chan3 : 11;
  unsigned int chan4 : 11;
  unsigned int chan5 : 11;
  unsigned int chan6 : 11;
  unsigned int chan7 : 11;
  unsigned int chan8 : 11;
  unsigned int chan9 : 11;
  unsigned int chan10 : 11;
  unsigned int chan11 : 11;
  unsigned int chan12 : 11;
  unsigned int chan13 : 11;
  unsigned int chan14 : 11;
  unsigned int chan15 : 11;
  uint8_t dig_chan0 : 1;
  uint8_t dig_chan1 : 1;
  uint8_t dig_chan2 : 1;
  uint8_t dig_chan3 : 1;
  uint8_t dig_chan4 : 1;
  uint8_t dig_chan5 : 1;
  uint8_t dig_chan6 : 1;
  uint8_t dig_chan7 : 1;
  uint8_t footer; // 0x00
} SbusPacket;

#define SBUS_DMA_BUFFER_SIZE (sizeof(SbusPacket) * 4)
DMA_RAM uint8_t sbus_dma_rxbuf[SBUS_DMA_BUFFER_SIZE];

DTCM_RAM uint8_t sbus_fifo_rx_buffer[SBUS_FIFO_BUFFERS * sizeof(RcPacket)];

uint32_t Sbus::init(
  // Driver initializers
  uint16_t sample_rate_hz,
  // UART initializers
  UART_HandleTypeDef * huart, USART_TypeDef * huart_instance, DMA_HandleTypeDef * hdma_uart_rx,
  uint32_t baud)
{
  sampleRateHz_ = sample_rate_hz;
  drdyPort_ = 0;      // do not use
  drdyPin_ = 0;       // do not use
  dtimeout_ = 100000; // 0.1 seconds
  timeout_ = 0;

  huart_ = huart;
  hdmaUartRx_ = hdma_uart_rx;

  // USART initialization begin (taken from STM32Cube Codegen)
  huart_->Instance = huart_instance;
  huart_->Init.BaudRate = baud;
  huart_->Init.WordLength = UART_WORDLENGTH_9B; // Including Parity
  huart_->Init.StopBits = UART_STOPBITS_2;
  huart_->Init.Parity = UART_PARITY_EVEN;
  huart_->Init.Mode = UART_MODE_TX_RX;
  huart_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_->Init.OverSampling = UART_OVERSAMPLING_16;
  huart_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart_->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart_->AdvancedInit.AdvFeatureInit =
    UART_ADVFEATURE_TXINVERT_INIT | UART_ADVFEATURE_RXINVERT_INIT;
  huart_->AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart_->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(huart_) != HAL_OK) return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_SetTxFifoThreshold(huart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_SetRxFifoThreshold(huart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    return DRIVER_HAL_ERROR;
  if (HAL_UARTEx_DisableFifoMode(huart_) != HAL_OK) return DRIVER_HAL_ERROR;
  // USART initialization end

  rxFifo_.init(SBUS_FIFO_BUFFERS, sizeof(RcPacket), sbus_fifo_rx_buffer);

  __HAL_UART_CLEAR_IDLEFLAG(huart_);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_IDLE);

  return DRIVER_OK;
}

bool Sbus::poll(void)
{
  // Check if we are timed-out
  if (time64.Us() > timeout_) {
    if ((((DMA_Stream_TypeDef *) (hdmaUartRx_)->Instance)->CR & DMA_SxCR_EN) != DMA_SxCR_EN) {
      __HAL_UART_CLEAR_IDLEFLAG(huart_); // this may be redundant with call to HAL_UART_Abort()
      __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
      HAL_UART_Abort(huart_); // flush any leftover crumbs.
      startDma();
    }
  }
  return 0;
}

bool Sbus::startDma(void)
{
  timeout_ = time64.Us() + dtimeout_; // 0.1 second timeout
  HAL_StatusTypeDef hal_status =
    HAL_UART_Receive_DMA(huart_, sbus_dma_rxbuf, SBUS_DMA_BUFFER_SIZE); // start next read
  return HAL_OK == hal_status;
}

void Sbus::endDma(void)
{
  drdy_ = time64.Us();

  RcPacket p;
  SbusPacket * sbus = (SbusPacket *) sbus_dma_rxbuf;

  if ((sbus->header == 0x0F) && (sbus->footer == 0x00)) {
    p.chan[0] = sbus->chan0;
    p.chan[1] = sbus->chan1;
    p.chan[2] = sbus->chan2;
    p.chan[3] = sbus->chan3;
    p.chan[4] = sbus->chan4;
    p.chan[5] = sbus->chan5;
    p.chan[6] = sbus->chan6;
    p.chan[7] = sbus->chan7;
    p.chan[8] = sbus->chan8;
    p.chan[9] = sbus->chan9;
    p.chan[10] = sbus->chan10;
    p.chan[11] = sbus->chan11;
    p.chan[12] = sbus->chan12;
    p.chan[13] = sbus->chan13;
    p.chan[14] = sbus->chan14;
    p.chan[15] = sbus->chan15;

    if (sbus->dig_chan0) p.chan[16] = 172;
    else p.chan[16] = 1811;
    if (sbus->dig_chan1) p.chan[17] = 172;
    else p.chan[17] = 1811;
    if (sbus->dig_chan2) p.chan[18] = 172;
    else p.chan[18] = 1811;
    if (sbus->dig_chan3) p.chan[19] = 172;
    else p.chan[19] = 1811;
    if (sbus->dig_chan4) p.chan[20] = 172;
    else p.chan[20] = 1811;
    if (sbus->dig_chan5) p.chan[21] = 172;
    else p.chan[21] = 1811;
    if (sbus->dig_chan6) p.chan[22] = 172;
    else p.chan[22] = 1811;
    if (sbus->dig_chan7) p.chan[23] = 172;
    else p.chan[23] = 1811;

    p.frameLost = sbus->dig_chan2;
    p.failsafeActivated = sbus->dig_chan3;
    for (int n = 0; n < RC_PACKET_CHANNELS; n++) p.chan[n] = (p.chan[n] - 172) / 1639.0;

    p.timestamp = time64.Us(); // usTime();
    p.drdy = drdy_;
    p.groupDelay = 9000; // Use one packet time latency for now.
    p.status = !(p.frameLost | p.failsafeActivated);

    lol_ = p.frameLost | p.failsafeActivated;

    rxFifo_.write((uint8_t *) &p, sizeof(p));
    timeout_ = drdy_ + dtimeout_; // Give it a second before we say it's lost
  }
  startDma();
}

bool Sbus::display(void)
{
  RcPacket p;
  char name[] = "Sbus (rc)";
  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    for (int i = 0; i < 8; i++) misc_printf("[%2u]:%4.0f, ", i + 1, (p.chan[i] * 100.0));
    misc_printf("%\n");
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    for (int i = 8; i < 16; i++) misc_printf("[%2u]:%4.0f, ", i + 1, (p.chan[i] * 100.0));
    misc_printf("%\n");
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    for (int i = 16; i < 24; i++) misc_printf("[%2u]:%4.0f, ", i + 1, (p.chan[i] * 100.0));
    misc_printf("%\n");
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("Frame Lost: %1u, Failsafe Activated: %1u, Status: %1u\n", p.frameLost,
                p.failsafeActivated, p.status);
    return 1;
  } else {
    misc_printf("%s\n", name);
    misc_printf("%s\n", name);
    misc_printf("%s\n", name);
    misc_printf("%s\n", name);
  }
  return 0;
}
