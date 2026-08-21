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

#include "CommonConfig.h"
#include "DoubleBuffer.h"

class STM32H7Board;


typedef struct __attribute__((__packed__))
{
  uint32_t rank;
  uint32_t chan;
  double scaleFactor;
  double offset;
} AdcChannelCfg;

typedef struct
{
  uint16_t ext_channels;
  uint16_t int_channels;
  uint16_t battery_voltage_index;
  uint16_t battery_current_index;
  uint16_t temperature_index;
  uint16_t vbat_index;
  uint16_t vref_index;
  int16_t vcc_index;
  const AdcChannelCfg * cfg;
  const char * const * names;
} AdcStructure;

/*
 *
 */
class Adc : public Status
{
public:
  static constexpr uint16_t MAX_ADC_CHANNELS = 16;
  static constexpr uint16_t MAX_ADC_NAME_LEN = 8;

  uint32_t init(uint16_t sample_rate_hz, ADC_HandleTypeDef * hadc_ext,
                ADC_TypeDef * adc_instance_ext, //
                ADC_HandleTypeDef * hadc_int,
                ADC_TypeDef * adc_instance_int, // This ADC has the calibration values
                const AdcStructure * init_structure);
  void register_callbacks(STM32H7Board & board, int32_t poll_phase_offset = 0);
  bool poll(uint64_t poll_offset);
  void adcConvCpltCallback(ADC_HandleTypeDef * hadc);

  bool display(void);
  bool isMy(ADC_HandleTypeDef * hadc) { return (hadcExt_ == hadc) || (hadcInt_ == hadc); }
  uint32_t channel_count() const { return channel_count_; }
  uint16_t battery_voltage_index() const { return battery_voltage_index_; }
  uint16_t battery_current_index() const { return battery_current_index_; }
  void setScaleFactor(uint16_t n, float scale_factor);

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size) == DoubleBufferStatus::OK; }

private:
  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size) == DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_ = 0;
  uint16_t channel_count_ = 0;
  uint16_t ext_channel_count_ = 0;
  uint16_t int_channel_count_ = 0;
  uint16_t battery_voltage_index_ = 0;
  uint16_t battery_current_index_ = 0;
  uint16_t temperature_index_ = 0;
  uint16_t vbat_index_ = 0;
  uint16_t vref_index_ = 0;
  int16_t vcc_index_ = -1;
  uint64_t drdy_ = 0;
  bool ext_read_ = false;
  bool int_read_ = false;

  uint32_t configChan(ADC_HandleTypeDef * hadc, ADC_ChannelConfTypeDef * sConfig, const AdcChannelCfg * cfg);
  uint32_t configAdc(ADC_HandleTypeDef * hadc, ADC_TypeDef * adc_instance, const AdcChannelCfg * cfg,
                     uint16_t cfg_channels);
  ADC_HandleTypeDef * hadcExt_ = nullptr;
  ADC_HandleTypeDef * hadcInt_ = nullptr;
  AdcChannelCfg * cfg_ = nullptr;
  const char * names_[MAX_ADC_CHANNELS * 2] = {};
  char name_storage_[MAX_ADC_CHANNELS * 2][MAX_ADC_NAME_LEN] = {};
};

#endif /* ADC_H_ */
