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
#include "CommonConfig.h"
#include "board.h"

//#define ADC1_MAX_CHANNELS_ADC (16)
//#define ADC3_MAX_CHANNELS_ADC (16)

#define ADC_MAX_CHANNELS (32)

#include <array>


#define ADC_NULL_STRING     "NULL    NA "
#define ADC_VBATT_STRING    "Vbatt   V  "
#define ADC_IBATT_STRING    "Ibatt   A  "
#define ADC_VSERVO_STRING   "Vservo  V  "
#define ADC_3V3_STRING      "FC 3v3  V  "
#define ADC_5V0_STRING      "FC 5v0  V  "
#define ADC_12V_STRING      "FC 12v  V  "
#define ADC_INT_TEMP_STRING "Temp    C  "
#define ADC_INT_VREF_STRING "Vref    V  "
#define ADC_INT_VBKU_STRING "Vbku    V  "
#define ADC_VRSSI_STRING    "RSSI    dB "


#define ADC_LABEL_LEN (12)

typedef struct
{
  uint32_t rank;
  uint32_t chan;
  float scale_factor;
  float offset;
  char label[ADC_LABEL_LEN];
} AdcChan;

typedef struct //__attribute__((__packed__))
{
  rosflight_firmware::PacketHeader header;
  double temperature;
  double vBku;
  double vRef;
  double voltage;
  double current;
  double volts[ADC_MAX_CHANNELS]; // holds two arrays
} AdcPacket;




class Adc : public Status
{
public:



  // we have to put this init() code here so the compiler can do the template thing
  template <size_t adc1_len, size_t adc3_len>
  uint32_t init(
      uint16_t sample_rate_hz,
      std::array<AdcChan,adc1_len> adc1_config,
      std::array<AdcChan,adc3_len> adc3_config
  ){
    snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Adc");
    initializationStatus_ = DRIVER_OK;
    sampleRateHz_ = sample_rate_hz;

    std::copy(adc1_config.begin(), adc1_config.end(), adc1_chan_.begin());
    std::copy(adc3_config.begin(), adc3_config.end(), adc3_chan_.begin());
    adc1_len_ = adc1_len;
    adc3_len_ = adc3_len;

    // TODO if

    // count up the channels on each adc
    extern uint8_t * adc_double_buffer_ptr;
    double_buffer_.init(adc_double_buffer_ptr, 2 * sizeof(AdcPacket));

    if (DRIVER_OK != configAdc(&hadc1)) {
      initializationStatus_ = DRIVER_HAL_ERROR;
    }
    if (DRIVER_OK != configAdc(&hadc3)) {
      initializationStatus_ |= DRIVER_HAL_ERROR;
    }
    // Figure out some indices...

    adcBattIIndex_ = 0xFFFF;
    adcBattVIndex_ = 0xFFFF;
    adc3VrefIndex_ = 0xFFFF;
    adc3BkuIndex_  = 0xFFFF;
    adc3TempIndex_ = 0xFFFF;

    for(uint32_t i=0;i< adc1_len_; i++) {
      if (strncmp(ADC_VBATT_STRING,adc1_chan_[i].label ,ADC_LABEL_LEN)==0) adcBattVIndex_=i;
      else if (strncmp(ADC_IBATT_STRING,adc1_chan_[i].label ,ADC_LABEL_LEN)==0) adcBattIIndex_=i;
    }

    // STM32 temp, vref, and vbat are only on adc3
    for (uint32_t i = 0; i < adc3_len_; i++) {

      if (strncmp(ADC_VBATT_STRING,adc3_chan_[i].label ,ADC_LABEL_LEN)==0) adcBattVIndex_=i+adc1_len_;
      else if (strncmp(ADC_IBATT_STRING,adc3_chan_[i].label ,ADC_LABEL_LEN)==0) adcBattIIndex_=i+adc1_len_;

      // STM32 temp, vref, and vbat are only on adc3, so no offset
      if (adc3_chan_[i].chan==ADC_CHANNEL_TEMPSENSOR) { adc3TempIndex_ = i;}
      else if (adc3_chan_[i].chan==ADC_CHANNEL_VREFINT) { adc3VrefIndex_ = i;}
      else if (adc3_chan_[i].chan==ADC_CHANNEL_VBAT) { adc3BkuIndex_ = i;}
    }

    return initializationStatus_;
  }

  bool poll(void) { return false; };
  bool poll(uint64_t poll_offset);
  void endDma(ADC_HandleTypeDef * hadc);

  bool display(void);
  // always true:
  bool isMy(ADC_HandleTypeDef * hadc) { return (&hadc1 == hadc) || (&hadc3 == hadc); }
  void setScaleFactor(uint16_t n, float scale_factor, float offset);

  bool read(uint8_t * data, uint16_t size) { return double_buffer_.read(data, size)==DoubleBufferStatus::OK; }

  void setCurrentScaleFactor(float scale_factor, float offset) {
   setScaleFactor(adcBattIIndex_ , scale_factor, offset);
  }
  void setVoltageScaleFactor(float scale_factor, float offset) {
   setScaleFactor(adcBattVIndex_ , scale_factor, offset);
  }

  double num_channels(void) { return adc1_len_ + adc3_len_; }
  double num_channels1(void) { return adc1_len_; }
  double num_channels3(void) { return adc3_len_; }

  bool name_channel(uint32_t ch, char *name, char *unit) {
    if (ch < adc1_len_) {
      strncpy(name, adc1_chan_[ch].label,7);
      name[7] = 0;
      strncpy(unit, adc1_chan_[ch].label+8,3);
      unit[3]=0;
      return true;
    } else if (ch < adc1_len_+ adc3_len_) {
      strncpy(name, adc3_chan_[ch-adc1_len_].label,7);
      name[7] = 0;
      strncpy(unit, adc3_chan_[ch-adc1_len_].label+8,3);
      unit[3]=0;
      return true;
    } else {
      name[0] = 0;
      unit[0] = 0;
      return false;
    }
  }
private:

  std::array<AdcChan,16> adc1_chan_;
  uint32_t adc1_len_;
  std::array<AdcChan,16> adc3_chan_;
  uint32_t adc3_len_;

  uint16_t adcBattIIndex_ = 0;
  uint16_t adcBattVIndex_ = 0;
  uint16_t adc3VrefIndex_ = 0;
  uint16_t adc3BkuIndex_  = 0;
  uint16_t adc3TempIndex_ = 0;

  bool write(uint8_t * data, uint16_t size) { return double_buffer_.write(data, size)==DoubleBufferStatus::OK; }
  DoubleBuffer double_buffer_;
  uint16_t sampleRateHz_;
  uint64_t drdy_;

  //uint32_t configChan(ADC_HandleTypeDef * hadc, ADC_ChannelConfTypeDef * sConfig, AdcChannelCfg * cfg);
  uint32_t configAdc(ADC_HandleTypeDef * hadc);
//  ADC_HandleTypeDef *hadcExt_, *hadcInt_; // The shared SPI handle


  DTCM_RAM uint8_t adc_double_buffer[2 * sizeof(AdcPacket)];
  DTCM_RAM uint32_t adc_counts[ADC_MAX_CHANNELS];

};

#endif /* ADC_H_ */
