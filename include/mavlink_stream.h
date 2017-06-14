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

#pragma once

#include <stdint.h>

// type definitions
// Needs to match mavlink_streams variable in mavlink_stream.c
typedef enum
{
  MAVLINK_STREAM_ID_HEARTBEAT,
  MAVLINK_STREAM_ID_STATUS,

  MAVLINK_STREAM_ID_ATTITUDE,
  MAVLINK_STREAM_ID_IMU,
  MAVLINK_STREAM_ID_DIFF_PRESSURE,
  MAVLINK_STREAM_ID_BARO,
  MAVLINK_STREAM_ID_SONAR,
  MAVLINK_STREAM_ID_MAG,
  MAVLINK_STREAM_ID_OUTPUT_RAW,
  MAVLINK_STREAM_ID_RC_RAW,

  MAVLINK_STREAM_ID_LOW_PRIORITY,

  MAVLINK_STREAM_COUNT
} mavlink_stream_id_t;

extern uint32_t _loop_time_us;

// function declarations
void mavlink_stream(uint64_t time_us);
void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate);
void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us);
