/**
 ******************************************************************************
 * File     : Adc.h
 * Date     : Oct 3, 2023
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

#ifndef ADC_H_
#define ADC_H_

#include "DoubleBuffer.h"
#include "BoardConfig.h"


typedef struct __attribute__((__packed__))
{
  uint32_t rank;
  uint32_t chan;
  double scaleFactor;
  double offset;
} AdcChannelCfg;

/*
 *
 */
class Adc : public Status
{
public:
  uint32_t init(uint16_t sample_rate_hz, ADC_HandleTypeDef * hadc_ext,
                ADC_TypeDef * adc_instance_ext, //
                ADC_HandleTypeDef * hadc_int,
                ADC_TypeDef * adc_instance_int // This ADC has the calibration values
  );
  bool poll(void) { return false; };
  bool poll(uint64_t poll_offset);
  void endDma(ADC_HandleTypeDef * hadc);

  bool display(void);
  bool isMy(ADC_HandleTypeDef * hadc) { return (hadcExt_ == hadc) || (hadcInt_ == hadc); }
  void setScaleFactor(uint16_t n, float scale_factor);

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size)==DoubleBufferStatus::OK; }

private:
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size)==DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  uint64_t drdy_;


  uint32_t configChan(ADC_HandleTypeDef * hadc, ADC_ChannelConfTypeDef * sConfig, AdcChannelCfg * cfg);
  uint32_t configAdc(ADC_HandleTypeDef * hadc, ADC_TypeDef * adc_instance, AdcChannelCfg * cfg, uint16_t cfg_channels);
  ADC_HandleTypeDef *hadcExt_, *hadcInt_; // The shared SPI handle
  AdcChannelCfg * cfg_;                   // has ADC_SCALE_FACTOR_EXT & INT
};

#endif /* ADC_H_ */
