/**
 ******************************************************************************
 * File     : Adc.cpp
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

#include "BoardConfig.h"

#include "Adc.h"
#include "stm32_h7.hpp"

#include "Packets.h"
#include "Time64.h"
#include "misc.h"

#include <cstring>

extern Time64 time64;

#define ADC_DMA_BUF_SIZE_MAX (16 * sizeof(uint32_t)) // 16 channels is max for the ADC sequencer

DTCM_RAM uint8_t adc_double_buffer[2 * sizeof(AdcPacket)];

DTCM_RAM uint32_t adc_counts[Adc::MAX_ADC_CHANNELS] = {};

DMA_RAM uint32_t adc_dma_buf_ext[ADC_DMA_BUF_SIZE_MAX / 4];
BDMA_RAM uint32_t adc_dma_buf_int[ADC_DMA_BUF_SIZE_MAX / 4]; // Internal is always ADC3, BDMA

DATA_RAM AdcChannelCfg adc_cfg[Adc::MAX_ADC_CHANNELS] = {};

uint32_t Adc::init(uint16_t sample_rate_hz, ADC_HandleTypeDef * hadc_ext,
                   ADC_TypeDef * adc_instance_ext, //
                   ADC_HandleTypeDef * hadc_int,
                   ADC_TypeDef * adc_instance_int, // This ADC has the calibration values
                   const AdcStructure * init_structure)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Adc");
  initializationStatus_ = DRIVER_OK;

  if ((sample_rate_hz == 0) || (init_structure == nullptr) || (init_structure->cfg == nullptr)
      || (hadc_ext == nullptr) || (hadc_int == nullptr) || (adc_instance_ext == nullptr)
      || (adc_instance_int == nullptr)) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }

  sampleRateHz_ = sample_rate_hz;
  ext_channel_count_ = init_structure->ext_channels;
  int_channel_count_ = init_structure->int_channels;
  channel_count_ = ext_channel_count_ + int_channel_count_;
  battery_voltage_index_ = init_structure->battery_voltage_index;
  battery_current_index_ = init_structure->battery_current_index;
  temperature_index_ = init_structure->temperature_index;
  vbat_index_ = init_structure->vbat_index;
  vref_index_ = init_structure->vref_index;
  vcc_index_ = init_structure->vcc_index;
  hadcExt_ = hadc_ext;
  hadcInt_ = hadc_int;

  if ((channel_count_ == 0) || (channel_count_ > MAX_ADC_CHANNELS) || (channel_count_ > MAX_ADC_PACKET_CHANNELS)
      || (battery_voltage_index_ >= channel_count_)
      || (battery_current_index_ >= channel_count_) || (temperature_index_ >= channel_count_)
      || (vbat_index_ >= channel_count_) || (vref_index_ >= channel_count_)
      || ((vcc_index_ >= 0) && (static_cast<uint16_t>(vcc_index_) >= channel_count_))) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }

  memset(adc_counts, 0, sizeof(adc_counts));
  memset(adc_cfg, 0, sizeof(adc_cfg));
  memset(names_, 0, sizeof(names_));
  memset(name_storage_, 0, sizeof(name_storage_));
  for (uint16_t i = 0; i < channel_count_; i++) {
    adc_cfg[i] = init_structure->cfg[i];
  }
  if (init_structure->names != nullptr) {
    for (uint16_t i = 0; i < (2 * channel_count_); i++) {
      if (init_structure->names[i] != nullptr) {
        snprintf(name_storage_[i], sizeof(name_storage_[i]), "%s", init_structure->names[i]);
        names_[i] = name_storage_[i];
      }
    }
  }
  cfg_ = adc_cfg;
  ext_read_ = false;
  int_read_ = false;

  double_buffer_.init(adc_double_buffer, sizeof(adc_double_buffer));

  if (DRIVER_OK != configAdc(hadcExt_, adc_instance_ext, cfg_, ext_channel_count_)) {
    initializationStatus_ = DRIVER_HAL_ERROR;
  }
  if (DRIVER_OK != configAdc(hadcInt_, adc_instance_int, &(cfg_[ext_channel_count_]), int_channel_count_)) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
  }
  return initializationStatus_;
}

uint32_t Adc::configChan(ADC_HandleTypeDef * hadc, ADC_ChannelConfTypeDef * sConfig, const AdcChannelCfg * cfg)
{
  sConfig->Rank = cfg->rank;
  sConfig->Channel = cfg->chan;
  if (HAL_ADC_ConfigChannel(hadc, sConfig) != HAL_OK) return DRIVER_HAL_ERROR;
  return DRIVER_OK;
}

uint32_t Adc::configAdc(ADC_HandleTypeDef * hadc, ADC_TypeDef * adc_instance, const AdcChannelCfg * cfg,
                        uint16_t cfg_channels)
{
  // uint32_t clock_prescaler ADC_CLOCK_ASYNC_DIV256; // This is reset below
  uint32_t sampling_cycles = ADC_SAMPLETIME_810CYCLES_5;
  uint32_t conversion_cycles = 8; // not adjustable
  // ADC is being fed with 64 MHz which is divided by 2 to make the ADC clock.
  // The sample time in us = 1/(64MHz/2)*clock_prescalar*(sampling_cycles+conversion_cycles)*ADC_MAX*oversample_ratio

  uint32_t clock_prescaler = (64000000 / 2) / sampleRateHz_ / ((1621 + 2 * conversion_cycles) / 2)
    / ((ext_channel_count_ > int_channel_count_) ? ext_channel_count_ : int_channel_count_);
  if (clock_prescaler > 256) clock_prescaler = ADC_CLOCK_ASYNC_DIV256;      // ~39.3 ms
  else if (clock_prescaler > 128) clock_prescaler = ADC_CLOCK_ASYNC_DIV128; // ~19.6 ms
  else clock_prescaler = ADC_CLOCK_ASYNC_DIV64;                             // ~ 9.8 ms

  hadc->Instance = adc_instance;
  hadc->Init.ClockPrescaler = clock_prescaler;
  hadc->Init.Resolution = ADC_RESOLUTION_16B;
  hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.ContinuousConvMode = DISABLE;
  hadc->Init.NbrOfConversion = cfg_channels;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc->Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(hadc) != HAL_OK) return DRIVER_HAL_ERROR;

  /** Configure the ADC multi-mode. Only set on ADC1*/
  if (hadc->Instance == ADC1) {
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(hadc, &multimode) != HAL_OK) return DRIVER_HAL_ERROR;
  }

  /** Configure Channels */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.SamplingTime = sampling_cycles;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;

  for (uint16_t i = 0; i < cfg_channels; i++)
    if (configChan(hadc, &sConfig, &(cfg[i])) != DRIVER_OK) return DRIVER_HAL_ERROR;

  HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  return DRIVER_OK;
}

bool Adc::poll(uint64_t poll_counter)
{
  uint32_t poll_offset = (uint32_t) (poll_counter % (POLLING_FREQ_HZ / sampleRateHz_));

  if (poll_offset == 0) // launch a read
  {
    drdy_ = time64.Us();
    HAL_StatusTypeDef hal_status_ext = HAL_ADC_Start_DMA(hadcExt_, (uint32_t *) adc_dma_buf_ext, ext_channel_count_);
    HAL_StatusTypeDef hal_status_int = HAL_ADC_Start_DMA(hadcInt_, (uint32_t *) adc_dma_buf_int, int_channel_count_);
    return ((HAL_OK == hal_status_int) && (HAL_OK == hal_status_ext));
  }

  return false;
}

void Adc::adcConvCpltCallback(ADC_HandleTypeDef * hadc)
{
  if (hadc == hadcExt_) {
    memcpy(adc_counts, adc_dma_buf_ext, ext_channel_count_ * sizeof(uint32_t));
    ext_read_ = true;
  } else if (hadc == hadcInt_) {
    memcpy(&(adc_counts[ext_channel_count_]), adc_dma_buf_int, int_channel_count_ * sizeof(uint32_t));
    int_read_ = true;
  }

  if (ext_read_ && int_read_) {
    AdcPacket p = {};
    p.temperature = (double) (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)
        / (double) (*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR)
        * ((double) adc_counts[temperature_index_] - (double) *TEMPSENSOR_CAL1_ADDR)
      + (double) TEMPSENSOR_CAL1_TEMP;

    p.vRef = (double) VREFINT_CAL_VREF / 1000.0 * (double) (*VREFINT_CAL_ADDR) / (double) adc_counts[vref_index_];
    p.vBku = 4.0 * (double) adc_counts[vbat_index_] * p.vRef / 65535.0;


    double vcc = p.vRef;
    if (vcc_index_ >= 0) {
      vcc = (double) (adc_counts[vcc_index_] & 0xFFFF) / 65535.0 * p.vRef * cfg_[vcc_index_].scaleFactor;
    }

    for (uint16_t i = 0; i < channel_count_; i++) {
      p.volts[i] = ((double) (adc_counts[i] & 0xFFFF) / 65535.0 * vcc - cfg_[i].offset) * cfg_[i].scaleFactor;
    }
    p.volts[vref_index_] = p.vRef;
    p.volts[vbat_index_] = p.vBku;
    p.volts[temperature_index_] = p.temperature;
    p.battery_voltage = p.volts[battery_voltage_index_];
    p.battery_current = p.volts[battery_current_index_];

    p.header.complete = time64.Us();
    p.header.timestamp = (drdy_+p.header.complete)/2;
    write((uint8_t *) &p, sizeof(p));
    ext_read_ = false;
    int_read_ = false;
  }
}

bool Adc::display(void)
{
  AdcPacket p;
  char name[] = "Adc (adc)";

  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.header);
    misc_printf("\n");

//    misc_printf("  %-8s : ", "STM");
//    misc_f32(3.0, 3.6, p.vBku, "Vrtc", "%5.1f", "V");               //
//    misc_f32(3.3 / 1.02, 3.3 * 1.02, p.vRef, "Vref", "%5.1f", "V"); //
//    misc_f32(18.0, 50.0, p.temperature, "Temp", "%5.1f", "C");      //
//    misc_printf("\n");
//
//    misc_printf("  %-8s : ", "Pwr");
//    misc_f32(14.0 / 1.02, 25.2 * 1.02, p.battery_voltage, "BattV", "%5.1f", "V");
//    misc_f32(0.1, 1.0, p.battery_current, "BattI", "%5.1f", "A");
//    misc_printf("\n");

    for(int i=0;i< (int)channel_count_; i++)
    {
      misc_printf("%-16s %5.1f %-s\n", (names_[2 * i] != nullptr) ? names_[2 * i] : "", p.volts[i], (names_[2 * i + 1] != nullptr) ? names_[2 * i + 1] : "");
    }
    return 1;
  } else {
    misc_printf("%s\n", name);
//    misc_printf("  STM\n");
//    misc_printf("  Pwr\n");
    for(int i=0;i< (int)channel_count_; i++)
    {
      misc_printf("%-16s\n", (names_[2 * i] != nullptr) ? names_[2 * i] : "");
    }
  }
  return 0;
}

void Adc::setScaleFactor(uint16_t n, float scale_factor)
{
  if (n < channel_count_) cfg_[n].scaleFactor = scale_factor;
}

void Adc::register_callbacks(STM32H7Board & board, int32_t poll_phase_offset)
{
  board.register_poll_client(this, poll_phase_offset);
  board.register_adc_client(this);
}
