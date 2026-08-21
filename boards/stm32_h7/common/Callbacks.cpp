/**
 ******************************************************************************
 * File     : Callbacks.cpp
 *
 * Date     : Sep 27, 2023
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
#include "stm32_h7.hpp"
extern STM32H7Board stm32_h7_board;

#include "Time64.h"
extern Time64 time64;

#include "Callbacks.h"

#include "BoardConfig.h"

#include "Polling.h"

void STM32H7Callbacks::clear_all()
{
  poll_client_len_ = 0;
  exti_client_len_ = 0;
  spi_client_len_ = 0;
  i2c_client_len_ = 0;
  adc_client_len_ = 0;
  cdc_client_len_ = 0;
  sd_client_len_ = 0;
  uart_rxcplt_client_len_ = 0;
  uart_rxisr_client_len_ = 0;
  uart_txcplt_client_len_ = 0;
}

void STM32H7Callbacks::register_poll_client(
  void * context, void (*callback)(void * context, uint64_t poll_counter), int32_t phase_offset)
{
  if (poll_client_len_ >= POLL_CLIENTS_MAX_LEN) return;

  PollClient & client = poll_clients_[poll_client_len_++];
  client.callback = callback;
  client.context = context;
  client.phase_offset = phase_offset;
}

void STM32H7Callbacks::register_exti_client(
  void * context, bool (*matches)(void * context, uint16_t exti_pin), void (*callback)(void * context))
{
  if (exti_client_len_ >= EXTI_CLIENTS_MAX_LEN) return;

  ExtiClient & client = exti_clients_[exti_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_spi_client(
  void * context, bool (*matches)(void * context, SPI_HandleTypeDef * hspi), void (*callback)(void * context))
{
  if (spi_client_len_ >= SPI_CLIENTS_MAX_LEN) return;

  SpiClient & client = spi_clients_[spi_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_i2c_client(
  void * context, bool (*matches)(void * context, I2C_HandleTypeDef * hi2c), void (*callback)(void * context))
{
  if (i2c_client_len_ >= I2C_CLIENTS_MAX_LEN) return;

  I2cClient & client = i2c_clients_[i2c_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_adc_client(
  void * context, bool (*matches)(void * context, ADC_HandleTypeDef * hadc),
  void (*callback)(void * context, ADC_HandleTypeDef * hadc))
{
  if (adc_client_len_ >= ADC_CLIENTS_MAX_LEN) return;

  AdcClient & client = adc_clients_[adc_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_cdc_client(
  void * context, bool (*matches)(void * context, uint8_t chan),
  void (*receive_callback)(void * context, uint8_t * buffer, uint16_t size),
  void (*transmit_cplt_callback)(void * context))
{
  if (cdc_client_len_ >= CDC_CLIENTS_MAX_LEN) return;

  CdcClient & client = cdc_clients_[cdc_client_len_++];
  client.matches = matches;
  client.receive_callback = receive_callback;
  client.transmit_cplt_callback = transmit_cplt_callback;
  client.context = context;
}

void STM32H7Callbacks::register_sd_client(
  void * context, bool (*matches)(void * context, SD_HandleTypeDef * hsd), void (*txcplt_callback)(void * context),
  void (*rxcplt_callback)(void * context))
{
  if (sd_client_len_ >= SD_CLIENTS_MAX_LEN) return;

  SdClient & client = sd_clients_[sd_client_len_++];
  client.matches = matches;
  client.txcplt_callback = txcplt_callback;
  client.rxcplt_callback = rxcplt_callback;
  client.context = context;
}

void STM32H7Callbacks::register_uart_rxcplt_client(
  void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context))
{
  if (uart_rxcplt_client_len_ >= UART_RXCPLT_CLIENTS_MAX_LEN) return;

  UartClient & client = uart_rxcplt_clients_[uart_rxcplt_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_uart_rxisr_client(
  void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context))
{
  if (uart_rxisr_client_len_ >= UART_RXISR_CLIENTS_MAX_LEN) return;

  UartClient & client = uart_rxisr_clients_[uart_rxisr_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::register_uart_txcplt_client(
  void * context, bool (*matches)(void * context, UART_HandleTypeDef * huart), void (*callback)(void * context))
{
  if (uart_txcplt_client_len_ >= UART_TXCPLT_CLIENTS_MAX_LEN) return;

  UartClient & client = uart_txcplt_clients_[uart_txcplt_client_len_++];
  client.matches = matches;
  client.callback = callback;
  client.context = context;
}

void STM32H7Callbacks::dispatch_poll(uint64_t poll_counter)
{
  for (uint32_t i = 0; i < poll_client_len_; i++) {
    const PollClient & client = poll_clients_[i];
    uint64_t adjusted_poll_counter = poll_counter;

    if (client.phase_offset >= 0) {
      adjusted_poll_counter += static_cast<uint64_t>(client.phase_offset);
    } else {
      const uint64_t phase_offset = static_cast<uint64_t>(-client.phase_offset);
      if (poll_counter < phase_offset) continue;
      adjusted_poll_counter -= phase_offset;
    }

    client.callback(client.context, adjusted_poll_counter);
  }
}

void STM32H7Callbacks::dispatch_exti(uint16_t exti_pin)
{
  for (uint32_t i = 0; i < exti_client_len_; i++) {
    const ExtiClient & client = exti_clients_[i];
    if (!client.matches(client.context, exti_pin)) continue;
    client.callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_spi(SPI_HandleTypeDef * hspi)
{
  for (uint32_t i = 0; i < spi_client_len_; i++) {
    const SpiClient & client = spi_clients_[i];
    if (!client.matches(client.context, hspi)) continue;
    client.callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_i2c(I2C_HandleTypeDef * hi2c)
{
  for (uint32_t i = 0; i < i2c_client_len_; i++) {
    const I2cClient & client = i2c_clients_[i];
    if (!client.matches(client.context, hi2c)) continue;
    client.callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_adc(ADC_HandleTypeDef * hadc)
{
  for (uint32_t i = 0; i < adc_client_len_; i++) {
    const AdcClient & client = adc_clients_[i];
    if (!client.matches(client.context, hadc)) continue;
    client.callback(client.context, hadc);
  }
}

void STM32H7Callbacks::dispatch_cdc_receive(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  if (chan != 0) return;
  for (uint32_t i = 0; i < cdc_client_len_; i++) {
    const CdcClient & client = cdc_clients_[i];
    if (!client.matches(client.context, chan)) continue;
    client.receive_callback(client.context, buffer, size);
  }
}

void STM32H7Callbacks::dispatch_cdc_transmit_cplt(uint8_t chan)
{
  if (chan != 0) return;
  for (uint32_t i = 0; i < cdc_client_len_; i++) {
    const CdcClient & client = cdc_clients_[i];
    if (!client.matches(client.context, chan)) continue;
    client.transmit_cplt_callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_sd_txcplt(SD_HandleTypeDef * hsd)
{
  for (uint32_t i = 0; i < sd_client_len_; i++) {
    const SdClient & client = sd_clients_[i];
    if (!client.matches(client.context, hsd)) continue;
    client.txcplt_callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_sd_rxcplt(SD_HandleTypeDef * hsd)
{
  for (uint32_t i = 0; i < sd_client_len_; i++) {
    const SdClient & client = sd_clients_[i];
    if (!client.matches(client.context, hsd)) continue;
    client.rxcplt_callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_uart_rxcplt(UART_HandleTypeDef * huart)
{
  for (uint32_t i = 0; i < uart_rxcplt_client_len_; i++) {
    const UartClient & client = uart_rxcplt_clients_[i];
    if (!client.matches(client.context, huart)) continue;
    client.callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_uart_rxisr(UART_HandleTypeDef * huart)
{
  for (uint32_t i = 0; i < uart_rxisr_client_len_; i++) {
    const UartClient & client = uart_rxisr_clients_[i];
    if (!client.matches(client.context, huart)) continue;
    client.callback(client.context);
  }
}

void STM32H7Callbacks::dispatch_uart_txcplt(UART_HandleTypeDef * huart)
{
  for (uint32_t i = 0; i < uart_txcplt_client_len_; i++) {
    const UartClient & client = uart_txcplt_clients_[i];
    if (!client.matches(client.context, huart)) continue;
    client.callback(client.context);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
// High Rate (10kHz) Periodic Timer Interrupt Routine for Polling
//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  if (stm32_h7_board.polling_timer().is_my(htim)) // Filter out other timer interrupts.
  {
    static uint64_t poll_counter = 0;
    poll_counter++;
    stm32_h7_board.callbacks().dispatch_poll(poll_counter);
    #define ROLLOVER 500000 // us
    const uint32_t polling_period_us = stm32_h7_board.polling_timer().period_us();
    if ((polling_period_us != 0U) && (0 == poll_counter % (ROLLOVER / polling_period_us))) GRN_TOG; // Blink Green LED at 1 Hz.
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
// EXTI (Data Ready) Interrupts

void HAL_GPIO_EXTI_Callback(uint16_t exti_pin)
{
  stm32_h7_board.callbacks().dispatch_exti(exti_pin);
}

//////////////////////////////////////////////////////////////////////////////////////////
// SPI Tx complete callback
//
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)  // All spi dma tx interrupts are handled here.
//{
//}

// SPI Rx complete callback
void HAL_SPI_TxRxCpltCallback(
  SPI_HandleTypeDef * hspi) // All spi dma rx interrupts are handled here.
{
  stm32_h7_board.callbacks().dispatch_spi(hspi);
}
//////////////////////////////////////////////////////////////////////////////////////////
// I2C Rx complete callback

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
  stm32_h7_board.callbacks().dispatch_i2c(hi2c);
}

//////////////////////////////////////////////////////////////////////////////////////////
// UART Rx complete callbacks
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  stm32_h7_board.callbacks().dispatch_uart_rxcplt(huart);
}

void UART_RxIsrCallback(UART_HandleTypeDef * huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // Serial Idle Management
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (huart->hdmarx != 0) ((DMA_Stream_TypeDef *) (huart->hdmarx)->Instance)->CR &= ~DMA_SxCR_EN;
  } else {
    stm32_h7_board.callbacks().dispatch_uart_rxisr(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
  stm32_h7_board.callbacks().dispatch_uart_txcplt(huart);
}

//////////////////////////////////////////////////////////////////////////////////////////
// ADC Rx complete callback

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
  stm32_h7_board.callbacks().dispatch_adc(hadc);
}

//////////////////////////////////////////////////////////////////////////////////////////
// VCP

void CDC_Receive_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  stm32_h7_board.callbacks().dispatch_cdc_receive(chan, buffer, size);
}

void CDC_TransmitCplt_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  (void) buffer;
  (void) size;
  stm32_h7_board.callbacks().dispatch_cdc_transmit_cplt(chan);
}

//////////////////////////////////////////////////////////////////////////////////////////
// SD card
void HAL_SD_TxCpltCallback(SD_HandleTypeDef * hsd)
{
  stm32_h7_board.callbacks().dispatch_sd_txcplt(hsd);
}

//// This function is called if there is an error during the transmission
//void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
//{
//    // Handle error
//  if (varmint.sd_.isMy(hsd)) { varmint.sd_.errorDma(hsd,0); }
//}
//
//// optional: This function is invoked when the SD card is not ready for I/O operation
//void HAL_SD_CardErrorCallback(SD_HandleTypeDef *hsd)
//{
//    // Handle SD card-specific error
//  if (varmint.sd_.isMy(hsd)) { varmint.sd_.errorDma(hsd,1); }
//}
//
//// optional: This function is invoked when the SD card has been disconnected or reconnected
//void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
//{
//    // Transmission has been aborted
//  if (varmint.sd_.isMy(hsd)) { varmint.sd_.errorDma(hsd,2); }
//}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef * hsd)
{
  stm32_h7_board.callbacks().dispatch_sd_rxcplt(hsd);
}
