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

#include "board.h"

#include "mavlink.h"
#include "printf.h"

#define mavlink_log(severity, format, ...) do { \
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN]; \
  tfp_sprintf(text, format, ##__VA_ARGS__); \
/*  mavlink_msg_statustext_send(MAVLINK_COMM_0, severity, text); */ \
  } while (0)


#define mavlink_log_critical(format, ...) mavlink_log(MAV_SEVERITY_CRITICAL, format, ##__VA_ARGS__)
#define mavlink_log_error(format, ...)    mavlink_log(MAV_SEVERITY_ERROR,    format, ##__VA_ARGS__)
#define mavlink_log_warning(format, ...)  mavlink_log(MAV_SEVERITY_WARNING,  format, ##__VA_ARGS__)
#define mavlink_log_info(format, ...)     mavlink_log(MAV_SEVERITY_INFO,     format, ##__VA_ARGS__)


//#define mavlink_log_critical_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_critical(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

//#define mavlink_log_error_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_error(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

//#define mavlink_log_warning_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_warning(format, ##__VA_ARGS__); \
//    }\
//  } while(0)

//#define mavlink_log_info_throttle(delay_ms, format, ...) \
//  do\
//  {\
//    static uint32_t last_hit = 0; \
//    uint32_t now = clock_millis(); \
//    if (now - last_hit > delay_ms) \
//    {\
//      last_hit = now; \
//      mavlink_log_info(format, ##__VA_ARGS__); \
//    }\
//  } while(0)
