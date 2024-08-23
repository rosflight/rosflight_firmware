/**
 ******************************************************************************
 * File     : Eng094x.h
 * Date     : Mar 25, 2023
 ******************************************************************************
 *
 * Copyright (c) 2024, AeroVironment, Inc.
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

// Part number is ENG094X-L10G-M25125-RR-N

#ifndef ENG094X_H_
#define ENG094X_H_

#include <BoardConfig.h>
#include <Driver.h>
#include <Spi.h>

#define ENG094X_CMD_BYTES 3

typedef enum
{
  ENG094X_PITOT = 0,
  ENG094X_BARO
} eng094x_press;

class Eng094x : public Driver
{
  /**
     * \brief
     *
     *
     */
public:
  uint32_t init(
    // Driver initializers
    uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
    uint16_t drdy_pin,                                 // Reset GPIO Pin
    // SPI initializers
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Chip Select GPIO Port
    uint16_t cs_pin,                                  // Chip Select GPIO Pin
    eng094x_press type);

  bool poll(uint64_t poll_counter);
  PollingState state(uint64_t poll_counter);
  void endDma(void);
  bool display(void) override;

  bool startTxDma(void);
  void endTxDma(void);

  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
  bool isMy(SPI_HandleTypeDef * hspi) { return hspi == spi_.hspi(); }
  SPI_HandleTypeDef * hspi(void) { return spi_.hspi(); }

private:
  uint32_t readCfg(uint8_t address, Spi * spi);

  // SPI Stuff
  Spi spi_;
  PollingState spiState_;
  uint8_t cmdBytes_[ENG094X_CMD_BYTES];

  eng094x_press type_;
  double LIN_A_, LIN_B_, LIN_C_, LIN_D_, Es_, TC50H_, TC50L_;
  double osDig_, fss_, off_;
  uint8_t sensor_status_ready_;
  char name_[16]; // for display
};

#endif /* ENG094X_H_ */
