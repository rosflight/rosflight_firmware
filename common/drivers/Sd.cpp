/**
 ******************************************************************************
 * File     : Sd.cpp
 * Date     : Oct 5, 2023
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

#include <Driver.h>
#include <Sd.h>

#include <Time64.h>
#include <misc.h>
#include <string.h>

extern Time64 time64;

#define SD_MAXBLKS (5L)
#define SD_BLKSIZE (512L)
#define SD_BUFF_SIZE (SD_MAXBLKS * SD_BLKSIZE)
DMA_RAM uint8_t sd_rx_buf[SD_BUFF_SIZE];
DMA_RAM uint8_t sd_tx_buf[SD_BUFF_SIZE];

uint32_t Sd::init(SD_HandleTypeDef * hsd, SD_TypeDef * hsd_instance)
{
  initializationStatus_ = DRIVER_OK;
  hsd_ = hsd;
  hsd_->Instance = hsd_instance;
  hsd_->Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd_->Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd_->Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd_->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd_->Init.ClockDiv = 10;
  HAL_StatusTypeDef hal_status = HAL_SD_Init(hsd_);
  if (hal_status != HAL_OK) {
    misc_printf("SD Card Initialization failure 0x%02X\n", hal_status);
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;
  }
  HAL_SD_CardInfoTypeDef sd_info;
  hal_status = HAL_SD_GetCardInfo(hsd_, &sd_info);
  if (hal_status != HAL_OK) {
    misc_printf("SD Card Info Read failure 0x%02X\n", hal_status);
    initializationStatus_ |= DRIVER_HAL_ERROR;
    return initializationStatus_;

  } else {
    misc_printf("SD Card Capacity= %.0f GB\n",
                (double) sd_info.BlockNbr * (double) sd_info.BlockSize / 1024. / 1024. / 1024.);
  }
  return initializationStatus_;
}

bool Sd::read(uint8_t * dest, size_t len)
{
  uint16_t Nblocks = (len + SD_BLKSIZE - 1) / SD_BLKSIZE;
  //	printf("Nblocks = %u\n",Nblocks);
  //	if(Nblocks > sd_info.BlockNbr) Nblocks = sd_info.BlockNbr;
  if (Nblocks > SD_MAXBLKS) return 0; // too large and don't want to be here forever, throw an error
  HAL_StatusTypeDef hal_status;
  HAL_SD_CardStateTypeDef sd_state;

  uint64_t timeout = time64.Us() + 250000;
  while ((HAL_SD_CARD_TRANSFER != (sd_state = HAL_SD_GetCardState(hsd_))) && (timeout > time64.Us()))
    ;
  if (HAL_SD_CARD_TRANSFER != sd_state) return 0;

  hal_status = HAL_SD_ReadBlocks(hsd_, sd_rx_buf, 0, Nblocks, 250);
  HAL_SD_GetCardState(hsd_);
  if (hal_status != HAL_OK) return 0;
  memcpy(dest, sd_rx_buf, len);

  return 1;
}
bool Sd::write(uint8_t * src, size_t len)
{
  uint16_t Nblocks = (len + 511) / 512;
  //	printf("Nblocks = %u\n",Nblocks);
  //	if(Nblocks > sd_info.BlockNbr) Nblocks = sd_info.BlockNbr;
  if (Nblocks > SD_MAXBLKS) return 0; // too large and don't want to be here forever, throw an error
  HAL_StatusTypeDef hal_status;
  HAL_SD_CardStateTypeDef sd_state;

  uint64_t timeout = time64.Us() + 250000;
  while ((HAL_SD_CARD_TRANSFER != (sd_state = HAL_SD_GetCardState(hsd_))) && (timeout > time64.Us()))
    ;
  if (HAL_SD_CARD_TRANSFER != sd_state) return 0;

  memcpy(sd_tx_buf, src, len);
  hal_status = HAL_SD_WriteBlocks(hsd_, sd_tx_buf, 0, Nblocks, 250);
  HAL_SD_GetCardState(hsd_);
  if (hal_status != HAL_OK) return 0;

  return 1;
}
