/**
 ******************************************************************************
 * File     : GpsDriver.cpp
 * Date     : Jun 11, 2026
 ******************************************************************************
 *
 * Copyright (c) 2026, AeroVironment, Inc.
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

#include "GpsDriver.h"
#include "BoardConfig.h"
#include "Packets.h"
#include "Time64.h"
#include "misc.h"
#include <ctime>

extern Time64 time64;

bool GpsDriver::poll(void)
{
  if (time64.Us() > timeout_) {
    if ((((DMA_Stream_TypeDef *) (huart_->hdmarx)->Instance)->CR & DMA_SxCR_EN) != DMA_SxCR_EN) {
      __HAL_UART_CLEAR_IDLEFLAG(huart_);
      __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
      HAL_UART_Abort(huart_);
      startDma();
    }
  }
  return 0;
}

bool GpsDriver::displayGnss(const char * name)
{
  GnssPacket p;

  if (read((uint8_t *) &p, sizeof(p))) {
    static double lag = 0;
    if (p.header.complete > p.header.timestamp) {
      lag = (lag * 0.99 + 0.01 * (double) (p.header.complete - p.header.timestamp));
    }
    struct tm * gmt;
    time_t seconds = p.unix_seconds;
    gmt = gmtime(&seconds);

    misc_header((char *) name, p.header);
    misc_printf("| pps %10.6f s | ", (double) p.pps * 1e-6);

    misc_printf("%02u/%02u/%04u ", gmt->tm_mon + 1, gmt->tm_mday, gmt->tm_year + 1900);
    misc_printf("%02u:%02u:%02u.%09d | ", gmt->tm_hour, gmt->tm_min, gmt->tm_sec, p.unix_nanos);

    misc_printf("%14.8f deg %14.8f deg +/- %5.1f m | ", (double) p.lat, (double) p.lon, p.h_acc);

    misc_printf("%9.3f m msl +/- %8.3f m | ", p.height_msl, p.v_acc);

    misc_printf("%5.1f %5.1f %5.1f +/- %5.1f m/s | ", p.vel_n, p.vel_e, p.vel_d, p.speed_accy);

    misc_printf("numSV %02u | ", p.num_sat);
    misc_printf("Fix %02u | ", p.fix_type);
    misc_printf("dt %6.0lf us\n", lag);
  } else {
    misc_printf("%s\n", name);
  }

  return true;
}
