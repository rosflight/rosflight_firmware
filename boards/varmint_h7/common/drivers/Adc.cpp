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

#include "Packets.h"
#include "Time64.h"
#include "misc.h"

extern Time64 time64;

#define ADC_DMA_BUF_SIZE_INT (ADC_CHANNELS_INT * sizeof(uint32_t))
#define ADC_DMA_BUF_SIZE_EXT (ADC_CHANNELS_EXT * sizeof(uint32_t))
#define ADC_DMA_BUF_SIZE_MAX (16 * sizeof(uint32_t)) // 16 channels is max for the ADC sequencer

DTCM_RAM uint8_t adc_fifo_rx_buffer[ADC_FIFO_BUFFERS * sizeof(AdcPacket)];
DTCM_RAM uint32_t adc_counts[ADC_CHANNELS];

ADC_EXT_DMA_RAM uint32_t adc_dma_buf_ext[ADC_DMA_BUF_SIZE_MAX / 4];
ADC_INT_DMA_RAM uint32_t adc_dma_buf_int[ADC_DMA_BUF_SIZE_MAX / 4];

DATA_RAM AdcChannelCfg adc_cfg[ADC_CHANNELS] = ADC_CFG_CHANS_DEFINE;

uint32_t Adc::init(uint16_t sample_rate_hz, ADC_HandleTypeDef * hadc_ext,
                   ADC_TypeDef * adc_instance_ext, //
                   ADC_HandleTypeDef * hadc_int,
                   ADC_TypeDef * adc_instance_int // This ADC has the calibration values
)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Adc");
  initializationStatus_ = DRIVER_OK;
  sampleRateHz_ = sample_rate_hz;
  hadcExt_ = hadc_ext;
  hadcInt_ = hadc_int;
  cfg_ = adc_cfg;

  groupDelay_ = 1000000 / sampleRateHz_;

  rxFifo_.init(ADC_FIFO_BUFFERS, sizeof(AdcPacket), adc_fifo_rx_buffer);

  if (DRIVER_OK != configAdc(hadcExt_, adc_instance_ext, cfg_, ADC_CHANNELS_EXT)) {
    initializationStatus_ = DRIVER_HAL_ERROR;
  }
  if (DRIVER_OK != configAdc(hadcInt_, adc_instance_int, &(cfg_[ADC_CHANNELS_EXT]), ADC_CHANNELS_INT)) {
    initializationStatus_ |= DRIVER_HAL_ERROR;
  }
  return initializationStatus_;
}

uint32_t Adc::configChan(ADC_HandleTypeDef * hadc, ADC_ChannelConfTypeDef * sConfig, AdcChannelCfg * cfg)
{
  sConfig->Rank = cfg->rank;
  sConfig->Channel = cfg->chan;
  if (HAL_ADC_ConfigChannel(hadc, sConfig) != HAL_OK) return DRIVER_HAL_ERROR;
  return DRIVER_OK;
}

uint32_t Adc::configAdc(ADC_HandleTypeDef * hadc, ADC_TypeDef * adc_instance, AdcChannelCfg * cfg,
                        uint16_t cfg_channels)
{
  // uint32_t clock_prescaler ADC_CLOCK_ASYNC_DIV256; // This is reset below
  uint32_t sampling_cycles = ADC_SAMPLETIME_810CYCLES_5;
  uint32_t conversion_cycles = 8; // not adjustable
  // ADC is being fed with 64 MHz which is divided by 2 to make the ADC clock.
  // The sample time in us = 1/(64MHz/2)*clock_prescalar*(sampling_cycles+conversion_cycles)*ADC_MAX*oversample_ratio

  uint32_t clock_prescaler = (64000000 / 2) / sampleRateHz_ / ((1621 + 2 * conversion_cycles) / 2)
    / ((ADC_CHANNELS_EXT > ADC_CHANNELS_INT) ? ADC_CHANNELS_EXT : ADC_CHANNELS_INT);
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

  for (int i = 0; i < cfg_channels; i++)
    if (configChan(hadc, &sConfig, &(cfg[i])) != DRIVER_OK) return DRIVER_HAL_ERROR;

  HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  return DRIVER_OK;
}

bool Adc::poll(uint64_t poll_counter)
{
  uint32_t poll_offset = (uint32_t) (poll_counter % (POLLING_FREQ_HZ / ADC_HZ));

  if (poll_offset == 0) // launch a read
  {
    drdy_ = time64.Us();
    HAL_StatusTypeDef hal_status_ext = HAL_ADC_Start_DMA(hadcExt_, (uint32_t *) adc_dma_buf_ext, ADC_CHANNELS_EXT);
    HAL_StatusTypeDef hal_status_int = HAL_ADC_Start_DMA(hadcInt_, (uint32_t *) adc_dma_buf_int, ADC_CHANNELS_INT);
    return ((HAL_OK == hal_status_int) && (HAL_OK == hal_status_ext));
  }

  return false;
}

void Adc::endDma(ADC_HandleTypeDef * hadc)
{
  static bool int_read = 0, ext_read = 0;

  if (hadc == hadcExt_) {
    memcpy(adc_counts, adc_dma_buf_ext, ADC_CHANNELS_EXT * sizeof(uint32_t));
    ext_read = 1;
  } else if (hadc == hadcInt_) {
    memcpy(&(adc_counts[ADC_CHANNELS_EXT]), adc_dma_buf_int, ADC_CHANNELS_INT * sizeof(uint32_t));
    int_read = 1;
  }

  if (ext_read && int_read) {
    AdcPacket p;
    p.temperature = (double) (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)
        / (double) (*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR)
        * ((double) adc_counts[ADC_STM_TEMPERATURE] - (double) *TEMPSENSOR_CAL1_ADDR)
      + (double) TEMPSENSOR_CAL1_TEMP;

    p.vRef = (double) VREFINT_CAL_VREF / 1000.0 * (double) (*VREFINT_CAL_ADDR) / (double) adc_counts[ADC_STM_VREFINT];
    p.vBku = 4.0 * (double) adc_counts[ADC_STM_VBAT] * p.vRef / 65535.0;

#ifdef ADC_3V3
    double vcc = (double) (adc_counts[ADC_3V3] & 0xFFFF) / 65535.0 * p.vRef * cfg_[ADC_3V3].scaleFactor;
#else
    double vcc = p.vRef;
#endif
    for (int i = 0; i < ADC_CHANNELS; i++) {
      //    	p.volts[i] = ((double)(adc_counts[i]&0xFFFF)/65535.0 - cfg_[i].offset) * p.vRef * cfg_[i].scaleFactor;
      p.volts[i] = ((double) (adc_counts[i] & 0xFFFF) / 65535.0 * p.vRef - vcc * cfg_[i].offset) * cfg_[i].scaleFactor;
    }

    p.header.timestamp = time64.Us();
    p.drdy = drdy_;
    p.groupDelay = groupDelay_;
    rxFifo_.write((uint8_t *) &p, sizeof(p));
    ext_read = 0;
    int_read = 0;
  }
}

bool Adc::display(void)
{
  AdcPacket p;
  char name[] = "Adc (adc)";

  if (rxFifo_.readMostRecent((uint8_t *) &p, sizeof(p))) {
    misc_header(name, p.drdy, p.header.timestamp, p.groupDelay);
    misc_printf("\n");

    misc_printf("  %-8s : ", "STM");
    misc_f32(3.0, 3.6, p.vBku, "Vbku", "%5.1f", "V");               //
    misc_f32(3.3 / 1.02, 3.3 * 1.02, p.vRef, "Vref", "%5.1f", "V"); //
    misc_f32(18.0, 50.0, p.temperature, "Temp", "%5.1f", "C");      //
    misc_printf("\n");

    misc_printf("  %-8s : ", "Pwr");
    misc_f32(22.2 / 1.02, 22.2 * 1.02, p.volts[ADC_BATTERY_VOLTS], "BattV", "%5.1f", "V"); //
    misc_f32(0.1, 1.0, p.volts[ADC_BATTERY_CURRENT], "BattI", "%5.1f", "A");               //
    misc_printf("\n");

    misc_printf("  %-8s : ", "PS_FC");

#ifdef ADC_3V3
    misc_f32(3.3 / 1.02, 3.3 * 1.02, p.volts[ADC_3V3], "3V3_FC", "%5.1f", "V"); //
#endif
#ifdef ADC_3V3_CURRENT
    misc_f32(0.0, 1.0, p.volts[ADC_3V3_CURRENT], "3I3_FC", "%5.1f", "A"); //
#endif

#ifdef ADC_5V0
    misc_f32(5.0 / 1.02, 5.0 * 1.02, p.volts[ADC_5V0], "5V0_FC", "%5.1f", "V"); //
#endif
#ifdef ADC_5V0_CURRENT
    misc_f32(0.0, 1.0, p.volts[ADC_5V0_CURRENT], "5I0_FC", "%5.1f", "A"); //
#endif

#ifdef ADC_12V
    misc_f32(12 / 1.02, 12 * 1.02, p.volts[ADC_12V], "12V_FC", "%5.1f", "V"); //
#endif
#ifdef ADC_12V_CURRENT
    misc_f32(0.0, 1.0, p.volts[ADC_12V_CURRENT], "12I_FC", "%5.1f", "A"); //
#endif

#ifdef ADC_SERVO_VOLTS
    misc_f32(8.2 / 1.02, 8.2 * 1.02, p.volts[ADC_SERVO_VOLTS], "ServoV", "%5.1f", "V"); //
#endif
#ifdef ADC_SERVO_CURRENT
    misc_f32(0.0, 2.0, p.volts[ADC_SERVO_CURRENT], "ServoI", "%5.1f", "A"); //
#endif
    misc_printf("\n");

    misc_printf("  %-8s : ", "PS_CC");
#ifdef ADC_CC_3V3
    misc_f32(3.3 / 1.02, 3.3 * 1.02, p.volts[ADC_CC_3V3], "3V3_CC", "%5.1f", "V"); //
#endif
#ifdef ADC_CC_3V3_CURRENT
    misc_f32(0.0, 2.0, p.volts[ADC_CC_3V3_CURRENT], "3I3_CC", "%5.1f", "A"); //
#endif

#ifdef ADC_CC_5V0
    misc_f32(5.0 / 1.02, 5.0 * 1.02, p.volts[ADC_CC_5V0], "5V0_CC", "%5.1f", "V"); //
#endif
#ifdef ADC_CC_5V0_CURRENT
    misc_f32(0.0, 2.0, p.volts[ADC_CC_5V0_CURRENT], "5I0_CC", "%5.1f", "A"); //
#endif

    misc_printf("\n");

    misc_printf("  %-8s : ", "ADC");

#ifdef ADC_J107_ADC_1
    misc_f32(0, 3.33 * 1.02, p.volts[ADC_J107_ADC_1], "J107-1", "%5.1f", "V"); //
#endif
#ifdef ADC_J107_ADC_2
    misc_f32(0, 3.33 * 1.02, p.volts[ADC_J107_ADC_2], "J107-2", "%5.1f", "V"); //
#endif
#ifdef ADC_J109_ADC_3
    misc_f32(0, 3.33 * 1.02, p.volts[ADC_J109_ADC_3], "J109-3", "%5.1f", "V"); //
#endif
#ifdef ADC_J109_ADC4
    misc_f32(0, 3.33 * 1.02, p.volts[ADC_J109_ADC4], "J109-4", "%5.1f", "V"); //
#endif
#ifdef ADC_RSSI_V                                                        // Pixracer Pro
    misc_f32(0, 3.33 * 1.02, p.volts[ADC_RSSI_V], "RSSI", "%5.1f", "V"); //
#endif

    misc_printf("\n");

    return 1;
  } else {
    misc_printf("%s\n", name);
    misc_printf("  STM\n");
    misc_printf("  Pwr\n");
    misc_printf("  PS_FC\n");
    misc_printf("  PS_CC\n");
    misc_printf("  ADC\n");
  }
  return 0;
}

void Adc::setScaleFactor(uint16_t n, float scale_factor)
{
  if (n < ADC_CHANNELS_EXT + ADC_CHANNELS_INT) cfg_[n].scaleFactor = scale_factor;
}
