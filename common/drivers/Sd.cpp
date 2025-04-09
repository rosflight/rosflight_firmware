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

#include "Sd.h"
#include "Time64.h"
#include "misc.h"

#include <string.h>

extern Time64 time64;

#define SD_MAXBLKS (16L)
#define SD_BLKSIZE (512L)
#define SD_BUFF_SIZE (SD_MAXBLKS * SD_BLKSIZE)
SD_DMA_RAM uint8_t sd_rx_buf[SD_BUFF_SIZE];
SD_DMA_RAM uint8_t sd_tx_buf[SD_BUFF_SIZE];

uint32_t Sd::init(SD_HandleTypeDef * hsd, SD_TypeDef * hsd_instance)
{
  snprintf(name_, STATUS_NAME_MAX_LEN, "%s", "Sd");
  initializationStatus_ = DRIVER_OK;
  txComplete_ = false;
  rxComplete_ = false;

  hsd_ = hsd;
  HAL_SD_DeInit(hsd_);
  hsd_->Instance = hsd_instance;
  hsd_->Init.ClockEdge = SDMMC_CLOCK_EDGE_FALLING; // PTT Check this. SDMMC_CLOCK_EDGE_RISING;
  hsd_->Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd_->Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd_->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd_->Init.ClockDiv = 2;
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
    HAL_SD_CardStateTypeDef sd_state = HAL_SD_GetCardState(hsd_);
    misc_printf("SD Card Capacity= %.0f GB, State = %lu\n",
                (double) sd_info.BlockNbr * (double) sd_info.BlockSize / 1024. / 1024. / 1024., sd_state);
  }

  return initializationStatus_;
}

#define SD_CRC_BYTES sizeof(uint32_t)

bool Sd::read(uint8_t * dest, size_t len)
{
  uint16_t Nblocks = (len + SD_CRC_BYTES + SD_BLKSIZE - 1) / SD_BLKSIZE;
  //  if(Nblocks > sd_info.BlockNbr) Nblocks = sd_info.BlockNbr;
  if (Nblocks > SD_MAXBLKS) { return 0; } // too large and don't want to be here forever, throw an error

  HAL_StatusTypeDef hal_status;

  rxComplete_ = false;
  hal_status = HAL_SD_ReadBlocks_DMA(hsd_, sd_rx_buf, 0, Nblocks);
  if (hal_status != HAL_OK) return false;

  uint64_t timeout = time64.Us() + 1000000; // Allow up to 1 second

  while (!rxComplete_ && (timeout > time64.Us())) {} // wait for DMA to complete

  if (rxComplete_) {
    // CRC must be set-up for byte sized data, i.e., hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES
    // I'm not bothering to code for the case of data in any other format
    if (hcrc.InputDataFormat != CRC_INPUTDATA_FORMAT_BYTES) { return false; }
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *) sd_rx_buf,
                                     len); // len Assumes hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES
    uint8_t * crc2 = (uint8_t *) (&crc);
    uint8_t * crc1 = (uint8_t *) (&(sd_rx_buf[len]));
    if ((crc1[0] != crc2[0]) || (crc1[1] != crc2[1]) || (crc1[2] != crc2[2]) || (crc1[3] != crc2[3])) { return false; }
    memcpy(dest, sd_rx_buf, len);
  }

  return rxComplete_;
}

bool Sd::write(uint8_t * src, size_t len)
{

  uint16_t Nblocks = (len + SD_CRC_BYTES + SD_BLKSIZE - 1) / SD_BLKSIZE;
  //  if(Nblocks > sd_info.BlockNbr) Nblocks = sd_info.BlockNbr;
  if (Nblocks > SD_MAXBLKS) return 0; // too large and don't want to be here forever, throw an error

  HAL_StatusTypeDef hal_status;

  txComplete_ = false;
  memcpy(sd_tx_buf, src, len);
  // CRC must be set-up for byte sized data hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES
  // I'm not bothering to code for the case of data in any other format
  if (hcrc.InputDataFormat != CRC_INPUTDATA_FORMAT_BYTES) { return false; }
  uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *) sd_tx_buf, len);
  uint8_t * crc2 = (uint8_t *) (&crc);
  sd_tx_buf[len] = crc2[0];
  sd_tx_buf[len + 1] = crc2[1];
  sd_tx_buf[len + 2] = crc2[2];
  sd_tx_buf[len + 3] = crc2[3];

  hal_status = HAL_SD_WriteBlocks_DMA(hsd_, sd_tx_buf, 0, Nblocks);
  if (hal_status != HAL_OK) return false;

  uint64_t timeout = time64.Us() + 100000; // Allow up to 100ms

  while (!txComplete_ && (timeout > time64.Us())) {} // wait for DMA to complete

  return txComplete_;
}
