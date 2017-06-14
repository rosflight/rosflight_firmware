/* 
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 */
#ifdef __cplusplus
extern "C" {
#endif

#pragma once

typedef enum
{
  ARMED = 0x01,
  FAILSAFE = 0x2,
} armed_state_t;
extern armed_state_t _armed_state;


typedef enum
{
  ERROR_NONE = 0x00,
  ERROR_INVALID_MIXER = 0x01,
  ERROR_IMU_NOT_RESPONDING = 0x02,
  ERROR_RC_LOST = 0x04,
  ERROR_UNHEALTHY_ESTIMATOR = 0x08,
  ERROR_TIME_GOING_BACKWARDS = 0x10,
  ERROR_UNCALIBRATED_IMU = 0x20,
} error_state_t;
extern error_state_t _error_state;

void init_mode(void);
bool check_mode();

#ifdef __cplusplus
}
#endif

