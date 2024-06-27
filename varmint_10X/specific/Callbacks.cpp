/**
 ******************************************************************************
 * File     : VarmintService.cpp
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
#include "stm32h753xx.h"

#include <Varmint.h>
extern Varmint varmint;

#include <Time64.h>
extern Time64 time64;

#include <Callbacks.h>

#include <BoardConfig.h>

#include <Polling.h>

//////////////////////////////////////////////////////////////////////////////////////////
// High Rate (10kHz) Periodic Timer Interrupt Routine for Polling
//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint64_t poll_counter = 0;
    poll_counter++;
    if (htim->Instance == POLL_HTIM_INSTANCE) // Filter out other timer interrupts.
    {
        varmint.baro_.poll(poll_counter);
        varmint.mag_.poll(poll_counter);
        varmint.pitot_.poll(poll_counter);

        varmint.rc_.poll();              // Restart if dead
        varmint.gps_.poll();             // Restart if dead
        varmint.telem_.poll();           // Check for new data packet to tx
        varmint.adc_.poll(poll_counter); // Start dma read
        varmint.vcp_.poll();             // Timeout

        if (0 == poll_counter % (POLLING_FREQ_HZ / 2))
            GRN_TOG; // Blink Green LED at 1 Hz.
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// EXTI (Data Ready) Interrupts

void HAL_GPIO_EXTI_Callback(uint16_t exti_pin)
{
    if (varmint.imu0_.isMy(exti_pin))
        varmint.imu0_.startDma();
    if (varmint.imu1_.isMy(exti_pin))
        varmint.imu1_.startDma();
    if (varmint.gps_.isMy(exti_pin))
        varmint.gps_.pps(time64.Us());
}

//////////////////////////////////////////////////////////////////////////////////////////
// SPI Tx complete callback
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)  // All spi dma tx interrupts are handled here.
//{
//}

// SPI Rx complete callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) // All spi dma rx interrupts are handled here.
{
    // do not use 'else if' since some of these share SPI
    if (varmint.imu0_.isMy(hspi))
        varmint.imu0_.endDma();
    if (varmint.imu1_.isMy(hspi))
        varmint.imu1_.endDma();
    if (varmint.mag_.isMy(hspi))
        varmint.mag_.endDma();
    if (varmint.baro_.isMy(hspi))
        varmint.baro_.endDma();
}

//////////////////////////////////////////////////////////////////////////////////////////
// I2C Rx complete callback

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (varmint.pitot_.isMy(hi2c))
        varmint.pitot_.endDma();
}

//////////////////////////////////////////////////////////////////////////////////////////
// UART Rx complete callbacks
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (varmint.rc_.isMy(huart))
        varmint.rc_.endDma();
    if (varmint.gps_.isMy(huart))
        varmint.gps_.endDma();
}

void UART_RxIsrCallback(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // Serial Idle Management
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        if (huart->hdmarx != 0)
            ((DMA_Stream_TypeDef *)(huart->hdmarx)->Instance)->CR &= ~DMA_SxCR_EN;
    }
    else
    {
        if (varmint.telem_.isMy(huart))
            varmint.telem_.rxIsrCallback(huart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (varmint.telem_.isMy(huart))
        varmint.telem_.txStart();
}

//////////////////////////////////////////////////////////////////////////////////////////
// ADC Rx complete callback

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (varmint.adc_.isMy(hadc))
        varmint.adc_.endDma(hadc);
}

//////////////////////////////////////////////////////////////////////////////////////////
// VCP

void CDC_Receive_Callback(uint8_t chan, uint8_t *buffer, uint16_t size)
{
    if (chan == 0)
        varmint.vcp_.rxCdcCallback(buffer, size);
}

void CDC_TransmitCplt_Callback(uint8_t chan, uint8_t *buffer, uint16_t size)
{
    if (chan == 0)
        varmint.vcp_.txCdcCallback();
}
