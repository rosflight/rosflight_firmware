/**
 ******************************************************************************
 * File     : Spi.h
 * Date     : Sep 26, 2023
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

#ifndef SPI_H_
#define SPI_H_

#include "stm32h7xx_hal.h"

#include <string.h> // for memset

#include <Time64.h>

#include <BoardConfig.h>
#include <Driver.h>

extern Time64 time64;

/*
 *
 */
#define WAIT2US time64.dUs(2)

class Spi
{
    /**
     * \brief
     *
     *
     */

  private:
    SPI_HandleTypeDef *hspi_;
    uint8_t *txBuffer_;
    uint8_t *rxBuffer_;

  public:
    // uint16_t size_;
    GPIO_TypeDef *port_;
    uint16_t pin_;
    uint32_t init(SPI_HandleTypeDef *hspi, uint8_t *tx_buffer, uint8_t *rx_buffer, GPIO_TypeDef *cs_port,
                  uint16_t cs_pin)
    {
        hspi_ = hspi;
        txBuffer_ = tx_buffer;
        rxBuffer_ = rx_buffer;
        port_ = cs_port;
        pin_ = cs_pin;
        return DRIVER_OK;
    }

    SPI_HandleTypeDef *hspi(void)
    {
        return hspi_;
    }

    uint8_t *endDma(void)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);

        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);

#if CACHE_MANAGEMENT_FUNCTIONS
        SCB_InvalidateDCache_by_Addr((uint32_t *)rxBuffer_, SPI_DMA_MAX_BUFFER_SIZE); // Force read from SRAM
#endif
        WAIT2US;
        return rxBuffer_;
    }

    void endTxDma(void)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);
        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    }

    HAL_StatusTypeDef startDma(uint8_t tx_byte, uint16_t size)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);

        memset(txBuffer_, 0, SPI_DMA_MAX_BUFFER_SIZE);
        txBuffer_[0] = tx_byte;
        //		memset(rxBuffer_,0xFF,SPI_DMA_MAX_BUFFER_SIZE);

#if CACHE_MANAGEMENT_FUNCTIONS
        SCB_CleanDCache_by_Addr((uint32_t *)txBuffer_, SPI_DMA_MAX_BUFFER_SIZE); // Force data from cache to SRAM
#endif

        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive_DMA(hspi_, txBuffer_, rxBuffer_, size);
        if ((HAL_OK != hal_status) && (software_nss))
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
        WAIT2US;

        return hal_status;
    }

    HAL_StatusTypeDef startDma(uint8_t *tx_bytes, uint16_t size)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);

        memset(txBuffer_, 0, SPI_DMA_MAX_BUFFER_SIZE);
        memcpy(txBuffer_, tx_bytes, size);
        //		memset(rxBuffer_,0xFF,SPI_DMA_MAX_BUFFER_SIZE);

#if CACHE_MANAGEMENT_FUNCTIONS
        SCB_CleanDCache_by_Addr((uint32_t *)txBuffer_, SPI_DMA_MAX_BUFFER_SIZE); // Force data from cache to SRAM
#endif

        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive_DMA(hspi_, txBuffer_, rxBuffer_, size);
        if ((HAL_OK != hal_status) && (software_nss))
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
        WAIT2US;

        return hal_status;
    }

    //	HAL_StatusTypeDef startTxDma(uint8_t tx_byte, uint16_t size)
    //	{
    //		bool software_nss 	= !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);
    //
    //		memset(txBuffer_,0,SPI_DMA_MAX_BUFFER_SIZE);
    //		txBuffer_[0] = tx_byte;
    //
    //		#if CACHE_MANAGEMENT_FUNCTIONS
    //				SCB_CleanDCache_by_Addr((uint32_t *)txBuffer_, SPI_DMA_MAX_BUFFER_SIZE) ; // Force data from cache
    // to SRAM
    //      #endif
    //
    //		if(software_nss) HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    //		HAL_StatusTypeDef hal_status = HAL_SPI_Transmit_DMA(hspi_, txBuffer_, size);
    //		if((HAL_OK!=hal_status) && (software_nss)) HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    //		time64.dUs(2);
    //
    //		return hal_status;
    //	}

    HAL_StatusTypeDef startTxDma(uint8_t *tx_bytes, uint16_t size)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);

        memset(txBuffer_, 0, SPI_DMA_MAX_BUFFER_SIZE);
        memcpy(txBuffer_, tx_bytes, size);

#if CACHE_MANAGEMENT_FUNCTIONS
        SCB_CleanDCache_by_Addr((uint32_t *)txBuffer_, SPI_DMA_MAX_BUFFER_SIZE); // Force data from cache to SRAM
#endif

        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_status = HAL_SPI_Transmit_DMA(hspi_, txBuffer_, size);
        if ((HAL_OK != hal_status) && (software_nss))
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
        WAIT2US;

        return hal_status;
    }

    HAL_StatusTypeDef rx(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size, uint16_t timeout_ms)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);
        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive(hspi_, tx_buffer, rx_buffer, size, timeout_ms);
        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
        WAIT2US;
        return hal_status;
    }

    HAL_StatusTypeDef tx(uint8_t *tx_buffer, uint16_t size, uint16_t timeout_ms)
    {
        bool software_nss = !(hspi_->Init.NSS == SPI_NSS_HARD_OUTPUT);
        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_status = HAL_SPI_Transmit(hspi_, tx_buffer, size, timeout_ms);
        if (software_nss)
            HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
        WAIT2US;
        return hal_status;
    }
};

#endif /* SPI_H_ */
