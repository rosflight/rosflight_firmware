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

#include <Adc.h>
#include <Time64.h>
#include <misc.h>

extern Time64 time64;

#define ADC_DMA_BUF_SIZE_INT (ADC_CHANNELS_INT * sizeof(uint32_t))
#define ADC_DMA_BUF_SIZE_EXT (ADC_CHANNELS_EXT * sizeof(uint32_t))

__attribute__((section("my_bdma_buffers")))
__attribute__((aligned(32))) static uint8_t adc_dma_buf_int[ADC_DMA_BUF_SIZE_INT] = {0};
__attribute__((section("my_dma_buffers")))
__attribute__((aligned(32))) static uint8_t adc_dma_buf_ext[ADC_DMA_BUF_SIZE_EXT] = {0};

__attribute__((section("my_buffers")))
__attribute__((aligned(32))) static uint8_t adc_fifo_rx_buffer[ADC_FIFO_BUFFERS * sizeof(AdcPacket)] = {0};

uint32_t Adc::init(uint16_t sample_rate_hz,
                   ADC_HandleTypeDef *hadcExt_ext,
                   ADC_TypeDef *adc_instance_ext,
                   ADC_HandleTypeDef *hadcExt_int,
                   ADC_TypeDef *adc_instance_int)
{
  sampleRateHz_ = sample_rate_hz;
  hadcExt_ = hadcExt_ext;
  hadcInt_ = hadcExt_int;

  groupDelay_ = 1000000 / sampleRateHz_;

  rxFifo_.init(ADC_FIFO_BUFFERS, sizeof(AdcPacket), adc_fifo_rx_buffer);

  uint32_t clock_prescaler = ADC_CLOCK_ASYNC_DIV64;
  uint32_t sampling_cycles = ADC_SAMPLETIME_810CYCLES_5;
  uint32_t conversion_cycles = 8;
  // ADC is being fed with 64 MHz which is divided by 2 to make the ADC clock.
  // The sample time in us = 1/(64MHz/2)*clock_prescalar*(sampling_cycles+conversion_cycles)*ADC_MAX*oversample_ratio

  clock_prescaler = (64000000 / 2) / sampleRateHz_ / ((1621 + 2 * conversion_cycles) / 2) / ADC_CHANNELS_EXT;
  if (clock_prescaler > 256)
    clock_prescaler = ADC_CLOCK_ASYNC_DIV256; // ~39.3 ms
  else if (clock_prescaler > 128)
    clock_prescaler = ADC_CLOCK_ASYNC_DIV128; // ~19.6 ms
  else
    clock_prescaler = ADC_CLOCK_ASYNC_DIV64; // ~ 9.8 ms

  {
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config 	*/
    hadcExt_->Instance = adc_instance_ext;
    hadcExt_->Init.ClockPrescaler = clock_prescaler;
    hadcExt_->Init.Resolution = ADC_RESOLUTION_16B;
    hadcExt_->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadcExt_->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadcExt_->Init.LowPowerAutoWait = DISABLE;
    hadcExt_->Init.ContinuousConvMode = DISABLE;
    hadcExt_->Init.NbrOfConversion = ADC_CHANNELS_EXT;
    hadcExt_->Init.DiscontinuousConvMode = DISABLE;
    hadcExt_->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadcExt_->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadcExt_->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadcExt_->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadcExt_->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadcExt_->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(hadcExt_) != HAL_OK)
      return DRIVER_HAL_ERROR;

    /** Configure the ADC multi-mode */
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(hadcExt_, &multimode) != HAL_OK)
      return DRIVER_HAL_ERROR;

    /** Configure Regular Channels */
    sConfig.SamplingTime = sampling_cycles;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;

    /** Configure Regular Channel */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(hadcExt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    /** Configure Regular Channel */
    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(hadcExt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    /** Configure Regular Channel	*/
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(hadcExt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    /** Configure Regular Channel	*/
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(hadcExt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    /** Configure Regular Channel */
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(hadcExt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;
    sConfig.Channel = ADC_CHANNEL_16;
    sConfig.Rank = ADC_REGULAR_RANK_6;
    if (HAL_ADC_ConfigChannel(hadcExt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;

    HAL_ADCEx_Calibration_Start(hadcExt_, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  }

  {
    ADC_ChannelConfTypeDef sConfig = {0};
    hadcInt_->Instance = adc_instance_int;
    hadcInt_->Init.ClockPrescaler = clock_prescaler;
    hadcInt_->Init.Resolution = ADC_RESOLUTION_16B;
    hadcInt_->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadcInt_->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadcInt_->Init.LowPowerAutoWait = DISABLE;
    hadcInt_->Init.ContinuousConvMode = DISABLE;
    hadcInt_->Init.NbrOfConversion = ADC_CHANNELS_INT;
    hadcInt_->Init.DiscontinuousConvMode = DISABLE;
    hadcInt_->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadcInt_->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadcInt_->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadcInt_->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadcInt_->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadcInt_->Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(hadcInt_) != HAL_OK)
      return DRIVER_HAL_ERROR;

    sConfig.SamplingTime = sampling_cycles;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;

    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(hadcInt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;

    sConfig.Channel = ADC_CHANNEL_VBAT;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(hadcInt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(hadcInt_, &sConfig) != HAL_OK)
      return DRIVER_HAL_ERROR;

    HAL_ADCEx_Calibration_Start(hadcInt_, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  }
  return DRIVER_OK;
}

bool Adc::poll(uint16_t poll_offset)
{
  if (poll_offset == 0) // launch a read
  {
    return startDma();
  }
  return false;
}

bool Adc::startDma(void)
{
  drdy_ = time64.Us();

  HAL_StatusTypeDef hal_status_int = HAL_ADC_Start_DMA(hadcInt_, (uint32_t *)adc_dma_buf_int, ADC_CHANNELS_INT);
  HAL_StatusTypeDef hal_status_ext = HAL_ADC_Start_DMA(hadcExt_, (uint32_t *)adc_dma_buf_ext, ADC_CHANNELS_EXT);

  return (HAL_OK == hal_status_int) && (HAL_OK == hal_status_ext);
}

void Adc::endDma(void)
{
#if USE_D_CACHE_MANAGEMENT_FUNCTIONS
  SCB_InvalidateDCache_by_Addr((uint32_t *)adc_dma_buf_ext, ADC_DMA_BUF_SIZE_EXT);
  SCB_InvalidateDCache_by_Addr((uint32_t *)adc_dma_buf_int, ADC_DMA_BUF_SIZE_INT);
#endif

  uint32_t *counts_ext = (uint32_t *)adc_dma_buf_ext;
  uint32_t *counts_int = (uint32_t *)adc_dma_buf_int;

  AdcPacket p;
  p.temperature = (double)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)
                      / (double)(*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR)
                      * ((double)counts_int[ADC_STM_TEMPERATURE] - (double)*TEMPSENSOR_CAL1_ADDR)
                  + (double)TEMPSENSOR_CAL1_TEMP;

  p.vRef = (double)VREFINT_CAL_VREF / 1000.0 * (double)*VREFINT_CAL_ADDR / (double)counts_int[ADC_STM_VREFINT];
  p.vBku = 4.0 * (double)counts_int[ADC_STM_VBAT] * p.vRef / 65535.0;

  for (int i = 0; i < ADC_CHANNELS_EXT; i++)
    p.volts_ext[i] = (double)counts_ext[i] * (p.vRef / 65535.0) * scaleFactor_[i];

  p.timestamp = time64.Us();
  p.drdy = drdy_;
  p.groupDelay = groupDelay_;
  rxFifo_.write((uint8_t *)&p, sizeof(p));
}

bool Adc::display(void)
{
  AdcPacket p;
  char name[] = "Adc (adc) ";
  if (rxFifo_.readMostRecent((uint8_t *)&p, sizeof(p)))
  {
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("  Batt  V   %5.2fV, I    %5.2fA, P    %5.2fW\n\r", p.volts_ext[ADC_BATTERY_VOLTS],
                p.volts_ext[ADC_BATTERY_CURR], p.volts_ext[ADC_BATTERY_VOLTS] * p.volts_ext[ADC_BATTERY_CURR]);
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("  PS's  3V3 %5.2fV, 5V0  %5.2fV, 12V  %5.2fV, Servo %5.2fV\n\r", p.volts_ext[ADC_JETSON_3V3],
                p.volts_ext[ADC_STM_5V0], p.volts_ext[ADC_STM_12V], p.volts_ext[ADC_SERVO_VOLTS]);
    misc_header(name, p.drdy, p.timestamp, p.groupDelay);
    misc_printf("  Temp    %7.2fC, Vbat %5.2fV, Vref %5.2fV\n\r", p.temperature, p.vBku, p.vRef);
    return 1;
  }
  else
  {
    misc_printf("%s\n\r", name);
    misc_printf("%s\n\r", name);
    misc_printf("%s\n\r", name);
  }
  return 0;
}

void Adc::setScaleFactor(uint16_t n, float scale_factor)
{
  if (n < ADC_CHANNELS_EXT)
    scaleFactor_[n] = scale_factor;
}
