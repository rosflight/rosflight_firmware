/**
 ******************************************************************************
 * File     : GpsDriver.h
 * Date     : Jun 6, 2026
 ******************************************************************************
 *
 * Copyright (c) 2026, AeroVironment, Inc.
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

#ifndef DRIVERS_GPS_DRIVER_H_
#define DRIVERS_GPS_DRIVER_H_

#include "DoubleBuffer.h"
#include "stm32h7xx_hal.h"
#include <cstdint>

/**
 * @class GpsDriver
 * @brief Base class for GPS receiver drivers with common UART/DMA infrastructure
 *
 * This class provides shared functionality for GPS drivers (Ubx, Liv4f, etc.)
 * and defines the callback interface required by board-specific Callbacks.cpp files.
 * Derived classes only need to implement protocol-specific parsing and initialization.
 */
class GpsDriver
{
public:
  virtual ~GpsDriver() = default;

  // Pure virtual methods - protocol-specific, must be implemented by derived classes

  /**
   * @brief DMA completion callback for UART receive
   * Called from HAL_UART_RxCpltCallback when GPS UART completes receive
   */
  virtual void endDma(void) = 0;

  /**
   * @brief Pulse-per-second timestamp callback
   * Called from HAL_GPIO_EXTI_Callback on PPS rising edge
   * @param pps_timestamp Microsecond timestamp of PPS edge
   */
  virtual void pps(uint64_t pps_timestamp) = 0;

  /**
   * @brief Start DMA receive operation (protocol-specific buffer)
   * @return true if DMA started successfully
   */
  virtual bool startDma(void) = 0;

  /**
   * @brief Display GPS data (to be overridden by concrete drivers)
   * @return true if data was displayed
   */
  virtual bool display(void) = 0;

  // Common methods - implemented in base class, available to all derived classes

  /**
   * @brief Check if EXTI pin belongs to this GPS instance
   * @param exti_pin GPIO pin number from EXTI callback
   * @return true if this GPS owns the pin
   */
  bool isMy(uint16_t exti_pin) { return ppsPin_ == exti_pin; }

  /**
   * @brief Check if UART handle belongs to this GPS instance
   * @param huart UART handle from HAL callback
   * @return true if this GPS owns the UART
   */
  bool isMy(UART_HandleTypeDef * huart) { return huart_ == huart; }

  /**
   * @brief Read GPS data from double buffer
   * @param data Destination buffer
   * @param size Number of bytes to read
   * @return true if read successful, false if no data or buffer error
   */
  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size) == DoubleBufferStatus::OK; }

  /**
   * @brief Get UART handle for this GPS instance
   * @return Pointer to UART handle
   */
  UART_HandleTypeDef * huart(void) { return huart_; }

  /**
   * @brief Periodic polling - restart DMA if dead
   * Checks for timeout and restarts DMA if it has stopped
   * @return Always returns false (status handled elsewhere)
   */
  bool poll(void);

protected:
  /**
   * @brief Display GPS data (common implementation for all drivers)
   * @param name Display name for this GPS instance
   * @return true if data was displayed
   */
  bool displayGnss(const char * name);
  // Common member variables accessible to derived classes
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  uint16_t ppsPin_;
  uint16_t ppsHz_;
  uint64_t timeout_;
  uint64_t dtimeout_;
  UART_HandleTypeDef * huart_;
  DMA_HandleTypeDef * hdmaUartRx_;
  uint32_t baud_;
  uint32_t baud_initial_;

  /**
   * @brief Write data to double buffer (for derived classes)
   * @param data Source buffer
   * @param size Number of bytes to write
   * @return true if write successful
   */
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size) == DoubleBufferStatus::OK; }
};

#endif /* DRIVERS_GPS_DRIVER_H_ */
