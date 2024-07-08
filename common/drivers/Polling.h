/**
 ******************************************************************************
 * File     : Polling.h
 *
 * Date     : June 20, 2024
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

#ifndef DRIVERS_POLLING_H_
#define DRIVERS_POLLING_H_

#include "stm32h7xx_hal.h"

// Polling Structure
typedef enum
{
    NULL_STATE = 0,
    IDLE_STATE,
    // DPS310 Baro Sensor
    DPS310_CMD_P,
    DPS310_DRDY_P,
    DPS310_RX_P,
    DPS310_CMD_T,
    DPS310_DRDY_T,
    DPS310_RX_T,
    DPS310_ERROR,
    // IIS2MDC Mat Sensor
    IIS2MDC_CMD,
    IIS2MDC_RX_H,
    IIS2MDC_RX_T,
    IIS2MDC_ERROR,
    // DLHR Pitot
    DLHR_CMD,
    DLHR_DRDY,
    DLHR_RX,
    DLHR_ERROR,
    // MS4525 Pitot
    MS4525_CMDRXSTART,
    MS4525_CMDRX,
    MS4525_CMDRXSEND,
    MS4525_ERROR,
    // IST8308 Mag
    IST8308_CMD,
    IST8308_TX,
    IST8308_RX,
    IST8308_ERROR,
    // AUAV Pitot
    AUAV_PITOT_CMD,
    AUAV_PITOT_RX,
    // AUAV Baro
    AUAV_BARO_CMD,
    AUAV_BARO_RX,
    AUAV_ERROR,
    // etc.
} PollingState;

typedef struct
{
    uint16_t index;
    PollingState state;
} PollingStateStruct;

PollingState PollingStateLookup(PollingStateStruct *ps, uint32_t size, uint32_t poll_index);

uint32_t InitPollTimer(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t channel);

#endif /* DRIVERS_POLLING_H_ */
