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

//////////////////////////////////////////////////////////////////////////////////////////
// High Rate (10kHz) Periodic Timer Interrupt Routine for Polling
//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  if (htim->Instance == POLL_HTIM_INSTANCE) // Filter out other timer interrupts.
  {

    static uint64_t poll_counter = 0;
    poll_counter++;
    stm32_h7_board.dispatch_poll(poll_counter);

    if (0 == poll_counter % (POLLING_FREQ_HZ / 2)) GRN_TOG; // Blink Green LED at 1 Hz.
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
// EXTI (Data Ready) Interrupts

void HAL_GPIO_EXTI_Callback(uint16_t exti_pin)
{
  stm32_h7_board.dispatch_exti(exti_pin);
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
  stm32_h7_board.dispatch_spi(hspi);
}
//////////////////////////////////////////////////////////////////////////////////////////
// I2C Rx complete callback

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
  stm32_h7_board.dispatch_i2c(hi2c);
}

//////////////////////////////////////////////////////////////////////////////////////////
// UART Rx complete callbacks
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  stm32_h7_board.dispatch_uart_rxcplt(huart);
}

void UART_RxIsrCallback(UART_HandleTypeDef * huart)
{
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // Serial Idle Management
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (huart->hdmarx != 0) ((DMA_Stream_TypeDef *) (huart->hdmarx)->Instance)->CR &= ~DMA_SxCR_EN;
  } else {
    stm32_h7_board.dispatch_uart_rxisr(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
  stm32_h7_board.dispatch_uart_txcplt(huart);
}

//////////////////////////////////////////////////////////////////////////////////////////
// ADC Rx complete callback

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
  stm32_h7_board.dispatch_adc(hadc);
}

//////////////////////////////////////////////////////////////////////////////////////////
// VCP

void CDC_Receive_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  stm32_h7_board.dispatch_cdc_receive(chan, buffer, size);
}

void CDC_TransmitCplt_Callback(uint8_t chan, uint8_t * buffer, uint16_t size)
{
  stm32_h7_board.dispatch_cdc_transmit_cplt(chan);
}

//////////////////////////////////////////////////////////////////////////////////////////
// SD card
void HAL_SD_TxCpltCallback(SD_HandleTypeDef * hsd)
{
  stm32_h7_board.dispatch_sd_txcplt(hsd);
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
  stm32_h7_board.dispatch_sd_rxcplt(hsd);
}
