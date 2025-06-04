/**
 ******************************************************************************
 * File     : Bmi088.h
 * Date     : Sep 28, 2023
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

#ifndef BMI088_H_
#define BMI088_H_

#include "Driver.h"
#include "Spi.h"

/*
 *
 */
class Bmi088 : public Driver
{
  /**
     * \brief
     *
     *
     */
public:
  uint32_t init(
    // Driver initializers
    uint16_t sample_rate, GPIO_TypeDef * drdy_port, // DRDY GPIO Port
    uint16_t drdy_pin,                              // DRDY GPIO Pin
    // SPI initializers
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * cs_port_a, // Chip Select GPIO Port
    uint16_t cs_pin_a,                                  // Chip Select GPIO Pin
    GPIO_TypeDef * cs_port_g,                           // Chip Select GPIO Port
    uint16_t cs_pin_g,                                  // Chip Select GPIO Pin
    // Sensor Specific
    uint8_t range_a, // 0,1,2,3,4 --> 2000,1000,500,250,125 deg/s
    uint8_t range_g  // // 0,1,2,3 --> 3,6,12,24g
  );

  void endDma(void);
  bool startDma(void);
  bool display(void) override;

  bool isMy(uint16_t exti_pin) { return drdyPin_ == exti_pin; }
  bool isMy(SPI_HandleTypeDef * hspi) { return hspi == spiA_.hspi(); }
  SPI_HandleTypeDef * hspi(void) { return spiA_.hspi(); }

private:
  // SPI Stuff
  Spi spiA_;
  Spi spiG_;
  uint16_t timeoutMs_;
  uint16_t seqCount_;
  // BMI088 Stuff
  uint8_t rangeA_, rangeG_;
  uint16_t syncCfgMode_;
  double accelRange_;

  void writeRegisterA(uint8_t address, uint8_t value);
  uint8_t readRegisterA(uint8_t address);
  void writeRegisterG(uint8_t address, uint8_t value);
  uint8_t readRegisterG(uint8_t address);
};

#endif /* BMI088_H_ */
