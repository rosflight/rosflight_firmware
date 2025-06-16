/**
 ******************************************************************************
 * File     : Adis165xx.h
 * Date     : Sep 20, 2023
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

#ifndef ADIS165XX_H_
#define ADIS165XX_H_

#include "BoardConfig.h"

#include "Spi.h"
#include "misc.h"
#include "Signal.h"

#define ADIS_OK (0x0000)

class Adis165xx : public Status , public MiscRotatable
{
public:
  uint32_t init(
    // Driver initializers
    uint16_t sample_rate_hz, GPIO_TypeDef * drdy_port, // Reset GPIO Port
    uint16_t drdy_pin,                                 // Reset GPIO Pin
    // SPI initializers
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port, // Chip Select GPIO Port
    uint16_t cs_pin,                                  // Chip Select GPIO Pin
    // ADIS165xx initializers
    GPIO_TypeDef * reset_port, // Reset GPIO Port
    uint16_t reset_pin,        // Reset GPIO Pin
    TIM_HandleTypeDef * htim, TIM_TypeDef * htim_instance, uint32_t htim_channel, uint32_t htim_period_us,
    const double *rotation
  );

  void endDma(void);
  bool startDma(void);
  bool display(void);
  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
  bool isMy(SPI_HandleTypeDef * hspi) { return hspi == spi_.hspi(); }
  SPI_HandleTypeDef * hspi(void) { return spi_.hspi(); }
  void set_rotation(double rotation[9]) { memcpy(rotation_,&rotation, 9*sizeof(double));}
  bool read(uint8_t * data, uint16_t size) { return (uint16_t)(signal_.read(data, size)==SignalStatus::OK); }

private:
  bool write(uint8_t * data, uint16_t size) { return (uint16_t)(signal_.write(data, size)==SignalStatus::OK); }

  Signal signal_;

  uint16_t sampleRateHz_;
  // SPI Stuff
  Spi spi_;
  uint16_t drdyPin_;
  uint64_t drdy_;

  uint16_t timeoutMs_;
  // ADIS165xx Stuff
  GPIO_TypeDef * resetPort_;
  uint16_t resetPin_;
  TIM_HandleTypeDef * htim_;
  uint32_t htimChannel_;
  void writeRegister(uint8_t address, uint16_t value);
  uint16_t readRegister(uint8_t address);
};

#endif /* ADIS165XX_H_ */
