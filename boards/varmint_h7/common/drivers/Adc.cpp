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

#include "CommonConfig.h"

#include "Adc.h"

#include "Packets.h"
#include "Time64.h"
#include "misc.h"
#include "Polling.h"

extern Time64 time64;
extern Polling polling;

#define ADC_DMA_BUF_SIZE_INT (ADC_CHANNELS_INT * sizeof(uint32_t))
#define ADC_DMA_BUF_SIZE_EXT (ADC_CHANNELS_EXT * sizeof(uint32_t))
#define ADC_DMA_BUF_SIZE_MAX (16 * sizeof(uint32_t)) // 16 channels is max for the ADC sequencer

DTCM_RAM uint8_t adc_double_buffer[2 * sizeof(AdcPacket)];
uint8_t * adc_double_buffer_ptr = adc_double_buffer;
//DTCM_RAM uint32_t adc_counts[ADC_CHANNELS];

DMA_RAM  uint32_t adc_dma_buf_1[ADC_DMA_BUF_SIZE_MAX / 4];
//DMA_RAM  uint32_t adc_dma_buf_2[ADC_DMA_BUF_SIZE_MAX / 4];
BDMA_RAM uint32_t adc_dma_buf_3[ADC_DMA_BUF_SIZE_MAX / 4]; // internal is adc3 is BDMA

//DATA_RAM AdcChannelCfg adc_cfg[ADC_CHANNELS] = ADC_CFG_CHANS_DEFINE;

//template <size_t adc1_len, size_t adc3_len>
//uint32_t Adc::init(
//    uint16_t sample_rate_hz,
//    std::array<AdcChan,adc1_len> adc1_config,
//    std::array<AdcChan,adc3_len> adc3_config
//)
//{
//  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Adc");
//  initializationStatus_ = DRIVER_OK;
//  sampleRateHz_ = sample_rate_hz;
//
//  std::copy(adc1_config.begin(), adc1_config.end(), adc1_chan_.begin());
//  std::copy(adc3_config.begin(), adc3_config.end(), adc3_chan_.begin());
//  adc1_len_ = adc1_len;
//  adc3_len_ = adc3_len;
//
//
//  // count up the channels on each adc
//
//  double_buffer_.init(adc_double_buffer, sizeof(adc_double_buffer));
//
//  if (DRIVER_OK != configAdc(&hadc1)) {
//    initializationStatus_ = DRIVER_HAL_ERROR;
//  }
//  if (DRIVER_OK != configAdc(&hadc3)) {
//    initializationStatus_ |= DRIVER_HAL_ERROR;
//  }
//  return initializationStatus_;
//}


uint32_t Adc::configAdc(ADC_HandleTypeDef * hadc)
{
  ADC_TypeDef * adc_instance = ADC3;
  uint32_t cfg_channels = adc3_len_;

  if (hadc == &hadc1) {
    adc_instance = ADC1;
    cfg_channels = adc1_len_;
  }


  // uint32_t clock_prescaler ADC_CLOCK_ASYNC_DIV256; // This is reset below
  uint32_t sampling_cycles = ADC_SAMPLETIME_810CYCLES_5;
  uint32_t conversion_cycles = 8; // not adjustable
  // ADC is being fed with 64 MHz which is divided by 2 to make the ADC clock.
  // The sample time in us = 1/(64MHz/2)*clock_prescalar*(sampling_cycles+conversion_cycles)*ADC_MAX*oversample_ratio

  uint32_t clock_prescaler = (64000000 / 2) / sampleRateHz_ / ((1621 + 2 * conversion_cycles) / 2)
    / ((adc1_len_ > adc3_len_) ? adc1_len_ : adc3_len_);
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

  if (hadc == &hadc1) {
    for (uint32_t i=0; i<adc1_len_; i++){
      sConfig.Rank = adc1_chan_[i].rank;
      sConfig.Channel = adc1_chan_[i].chan;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return DRIVER_HAL_ERROR;
    }
  } else if(hadc == &hadc3) {
    for (uint32_t i=0; i<adc3_len_; i++){
      sConfig.Rank = adc3_chan_[i].rank;
      sConfig.Channel = adc3_chan_[i].chan;
      if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) return DRIVER_HAL_ERROR;
    }
  }

  HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);


  return DRIVER_OK;
}


bool Adc::poll(uint64_t poll_counter)
{
  uint32_t poll_offset = polling.index(poll_counter, 1000000/sampleRateHz_); //(uint32_t) (poll_counter % (adc_us/POLLING_PERIOD_US));

  if (poll_offset == 0) // launch a read
  {
    drdy_ = time64.Us();

    uint32_t hal_status = HAL_OK;

    if( (adc1_len_>0) && (adc1_len_<=16) ) {
      hal_status |= HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_dma_buf_1, adc1_len_);
    }
    if( (adc3_len_>0) && (adc3_len_<=16) ) {
      hal_status |= HAL_ADC_Start_DMA(&hadc3, (uint32_t *) adc_dma_buf_3, adc3_len_);
    }

    return hal_status == HAL_OK;
  }

  return false;
}


void Adc::endDma(ADC_HandleTypeDef * hadc)
{
  static uint8_t read=0;

  if (hadc == &hadc1) { read  |= 0x01; }
  else if (hadc == &hadc3) { read |= 0x04;}

  if (read==0x05) {


    AdcPacket p;

    // adc3-specific internal sensors
    uint32_t *counts3 = (uint32_t *) adc_dma_buf_3;
    if (adc3TempIndex_ < adc3_len_)
    {
      p.temperature = (double) (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)
        / (double) (*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR)
        * ((double) (counts3[adc3TempIndex_] & 0xFFFF) - (double) *TEMPSENSOR_CAL1_ADDR)
        + (double) TEMPSENSOR_CAL1_TEMP;
    } else {
      p.temperature = -273.15; // C
    }

    if (adc3VrefIndex_ < adc3_len_)
    {
      p.vRef = (double) VREFINT_CAL_VREF / 1000.0 * (double) (*VREFINT_CAL_ADDR) / (double) (counts3[adc3VrefIndex_] & 0xFFFF);
    } else {
      p.vRef = 3.3; // V
    }

    if (adc3BkuIndex_ < adc3_len_)
    {
      p.vBku = 4.0 * (double) (counts3[adc3BkuIndex_] & 0xFFFF) / 65535.0 * p.vRef; // known offset and scale factor.
    } else {
      p.vBku = 0.0; // V
    }

    uint32_t *counts1 = (uint32_t *) adc_dma_buf_1;

    for(uint32_t i=0;i<adc1_len_;i++) {
      p.volts[i] = ((double) (counts1[i] & 0xFFFF) / 65535.0 * p.vRef - adc1_chan_[i].offset) * adc1_chan_[i].scale_factor;
    }

    for(uint32_t i=0;i<adc3_len_;i++) {
      p.volts[i+adc1_len_] = ((double) (counts3[i] & 0xFFFF) / 65535.0 * p.vRef - adc3_chan_[i].offset) * adc3_chan_[i].scale_factor;
    }
    uint32_t channels = adc1_len_ + adc3_len_;
    if (adcBattIIndex_< channels) p.current = p.volts[adcBattIIndex_]; else p.current = 0.0;
    if (adcBattVIndex_< channels) p.voltage = p.volts[adcBattVIndex_]; else p.voltage = 0.0;

    if (adc3BkuIndex_ < adc3_len_) p.volts[adc3BkuIndex_+adc1_len_] = p.vBku;
    if (adc3VrefIndex_ < adc3_len_) p.volts[adc3VrefIndex_+adc1_len_] = p.vRef;
    if (adc3TempIndex_ < adc3_len_) p.volts[adc3TempIndex_+adc1_len_] = p.temperature;

    p.header.complete = time64.Us();
    p.header.timestamp = (drdy_+p.header.complete)/2;
    write((uint8_t *) &p, sizeof(p));
    read = 0;
  }
}


bool Adc::display(void)
{
  AdcPacket p;
  char name[] = "Adc (adc)";

  if (read((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.header);
    misc_printf("\n");

    misc_printf("  %-8s : ", "STM");
    misc_f32(3.0       , 3.6       , p.vBku, "Vbku", "%5.1f", "V");
    misc_f32(3.3 / 1.02, 3.3 * 1.02, p.vRef, "Vref", "%5.1f", "V");
    misc_f32(18.0, 50.0, p.temperature, "Temp", "%5.1f", "C");
    misc_printf("\n");

    misc_printf("  %-8s : ", "Pwr");
    misc_f32(22.2 / 1.02, 22.2 * 1.02, p.voltage, "Vbatt", "%5.1f", "V");
    misc_f32(0.1, 1.0, p.current, "Ibatt", "%5.1f", "A");
    misc_printf("\n");

    misc_printf("  %-8s : ", "ADC1");
    for(uint32_t i = 0; i<num_channels1(); i++ ) {
      char label[8] = {0};
      char unit[8]  = {0};
      if(name_channel(i,&label[0],&unit[0])) {
        misc_printf("%-7s %4.1f %-3s |", label, p.volts[i], unit);
      }
    }
    misc_printf("\n");

    misc_printf("  %-8s : ", "ADC3");
    for(uint32_t i = num_channels1(); i<num_channels(); i++ ) {
      char label[8] = {0};
      char unit[8]  = {0};
      if(name_channel(i,&label[0],&unit[0])) {
        misc_printf("%-7s %4.1f %-3s |", label, p.volts[i], unit);
      }
    }
    misc_printf("\n");

    return 1;
  } else {
    misc_printf("%s\n", name);
    misc_printf("  STM\n");
    misc_printf("  Pwr\n");
    misc_printf("  ADC1\n");
    misc_printf("  ADC3\n");
  }
  return 0;
}


void Adc::setScaleFactor(uint16_t n, float scale_factor, float offset)
{
  if (n < adc1_len_ ) {
    adc1_chan_[n].offset = offset;
    adc1_chan_[n].scale_factor = scale_factor;
  } else if (n < (adc1_len_ + adc3_len_)) {
    adc3_chan_[n-adc1_len_].offset = offset;
    adc3_chan_[n-adc1_len_].scale_factor = scale_factor;
  }
}




