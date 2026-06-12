/**
 ******************************************************************************
 * File     : GpsNull.h
 * Date     : Jun 11, 2026
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

#ifndef DRIVERS_GPS_NULL_H_
#define DRIVERS_GPS_NULL_H_

#include "GpsDriver.h"

/**
 * @class GpsNull
 * @brief Null Object GPS driver that does nothing (used when no GPS detected)
 *
 * This implements the Null Object pattern to avoid null pointer checks throughout
 * the codebase. All methods are safe no-ops that return appropriate default values.
 */
class GpsNull : public Status, public GpsDriver
{
public:
  GpsNull() { snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "GpsNull"); }

  uint32_t init(uint16_t sample_rate_hz, GPIO_TypeDef * pps_port, uint16_t pps_pin,
                UART_HandleTypeDef * huart, USART_TypeDef * huart_instance,
                DMA_HandleTypeDef * hdma_uart_rx, uint32_t baud_desired)
  {
    (void)sample_rate_hz; (void)pps_port; (void)pps_pin;
    (void)huart; (void)huart_instance; (void)hdma_uart_rx; (void)baud_desired;
    initializationStatus_ = DRIVER_OK;
    return DRIVER_OK;
  }

  // GpsDriver interface - all no-ops
  void endDma(void) override {}
  void pps(uint64_t pps_timestamp) override { (void)pps_timestamp; }
  bool startDma(void) override { return true; }

  bool display(void) override { misc_printf("No GPS\n"); return false; }
};

#endif /* DRIVERS_GPS_NULL_H_ */
