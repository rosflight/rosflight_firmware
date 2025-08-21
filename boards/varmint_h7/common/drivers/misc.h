/**
 ******************************************************************************
 * File     : misc.h
 * Date     : Sep 23, 2023
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

#ifndef MISC_H_
#define MISC_H_

#include <stdint.h>
#include <stm32h7xx_hal.h>
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

//	Foreground Colors
enum class MiscColor
{
  NONE = 0,
  BLACK_FG = 30,
  RED_FG = 31,
  GREEN_FG = 32,
  YELLOW_FG = 33,
  BLUE_FG = 34,
  MAGENTA_FG = 35,
  CYAN_FG = 36,
  WHITE_FG = 37,
  //	Background Colors
  BLACK_BG = 40,
  RED_BG = 41,
  GREEN_BG = 42,
  YELLOW_BG = 43,
  BLUE_BG = 44,
  MAGENTA_BG = 45,
  CYAN_BG = 46,
  WHITE_BG = 47,
};

#define ASCII_ESC 27

void misc_printfc(MiscColor color_code, const char * format, ...);
void misc_clear();
void misc_home();
void misc_f32(float lo, float hi, float x, const char * pre, const char * number_format, const char * post);
void misc_x16(uint16_t match, uint16_t x, const char * pre);
void misc_i32(int32_t lo, int32_t hi, int32_t x, const char * pre, const char * number_format, const char * post);
void misc_u32(uint32_t lo, uint32_t hi, uint32_t x, const char * pre, const char * number_format, const char * post);

void misc_printf(const char * format, ...);
size_t misc_getline(uint8_t * line, size_t len);
void misc_header(char * name, rosflight_firmware::PacketHeader &header);
uint16_t misc_bytes_in_dma(DMA_HandleTypeDef * hdma_uart_rx, uint16_t dma_buffer_size);
void misc_exit_status(uint32_t status);


#ifdef __cplusplus
}
#endif

class MiscRotatable
{
public:
  void rotate(double *x) {
    double y[3];
    y[0] = x[0]*rotation_[0] + x[1]*rotation_[1] + x[2]*rotation_[2];
    y[1] = x[0]*rotation_[3] + x[1]*rotation_[4] + x[2]*rotation_[5];
    y[2] = x[0]*rotation_[6] + x[1]*rotation_[7] + x[2]*rotation_[8];
    x[0] = y[0];
    x[1] = y[1];
    x[2] = y[2];
  }
protected:
  double rotation_[9];
};

#endif /* MISC_H_ */
