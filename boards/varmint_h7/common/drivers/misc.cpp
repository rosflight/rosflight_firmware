/**
 ******************************************************************************
 * File     : misc.c
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

#include "BoardConfig.h"
#include "usb_device.h"
//#include "usbd_cdc_if.h"
#include "usbd_cdc_acm_if.h" // PTT

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "misc.h"

extern bool verbose;

extern "C" {
int __io_putchar(int ch)
{
#ifdef __HAVE_SWO__
  if (ch != '\r') { // don't send '\r'
    ITM_SendChar(ch);
  }
#else
  HAL_UART_Transmit(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFFFFFF);
#endif
  return 1;
}

int __io_getchar(void)
{
  uint8_t ch = 0;
#ifdef __HAVE_SWO__
  // ch = ITM_ReceiveChar(); // for SWO, not supported on STM32CubeIDE.
#else
  HAL_UART_Receive(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFF);
#endif
  return (int) ch;
}
}

// typedef enum
//{
//   USBD_OK = 0U,
//   USBD_BUSY,
//   USBD_EMEM,
//   USBD_FAIL,
// } USBD_StatusTypeDef;

// NOTE! Only use misc_printf for debugging since it blocks.
#define MAX_SPRINTF_CHARS 256
char misc_sprintf_buffer[MAX_SPRINTF_CHARS];

void misc_printf(const char * format, ...)
{

  if (!verbose) return;

  va_list argp;
  va_start(argp, format);
  //  vprintf(format, argp);
  size_t len = vsnprintf(misc_sprintf_buffer, MAX_SPRINTF_CHARS, format, argp);
  HAL_UART_Transmit(MISC_HUART, (uint8_t *) misc_sprintf_buffer, len, 0xFFFFFFFF);
  va_end(argp);

  //	uint8_t vcp_status=USBD_OK;
  //	va_list argp;
  //	va_start(argp, format);
  //	vsnprintf(misc_sprintf_buffer, MAX_SPRINTF_CHARS, format, argp);
  //
  //	while(USBD_OK!=(vcp_status=VCP_Transmit((uint8_t*)misc_sprintf_buffer, strlen(misc_sprintf_buffer))))
  //	{
  //	}
  //	va_end(argp);
}

void misc_clear()
{
  //  misc_printf("\033[H"); //  home
  misc_printf("\033[2J"); // clear
  misc_printf("\033[0m"); // clear colors
}

void misc_home() { misc_printf("\033[H"); }

void misc_printfc(MiscColor color_code, const char * format, ...)
{
  if (!verbose) return;

  misc_printf("\033[0;%02um", (uint8_t) color_code);

  va_list argp;
  va_start(argp, format);
  vprintf(format, argp);
  va_end(argp);

  misc_printf("\033[0m");
}

void misc_num(uint8_t fail, const char * pre, char * num_char, size_t num_len, const char * post)
{
  const size_t bufferlen = 16;
  char buffer[bufferlen];

  size_t len = 0;
  if (fail == 0) len = snprintf(buffer, bufferlen, "\033[0;41m%-7s", pre);      // Red
  else if (fail == 1) len = snprintf(buffer, bufferlen, "\033[0;42m%-7s", pre); // Green
  else len = snprintf(buffer, bufferlen, "\033[0m%-7s", pre);                   // Nothing

  HAL_UART_Transmit(MISC_HUART, (uint8_t *) buffer, len, 0xFFFFFFFF);

  HAL_UART_Transmit(MISC_HUART, (uint8_t *) num_char, num_len, 0xFFFFFFFF);

  len = snprintf(buffer, bufferlen, " %-4s\033[0m|", post);
  HAL_UART_Transmit(MISC_HUART, (uint8_t *) buffer, len, 0xFFFFFFFF);
}

// Alternate format
//
//void misc_num(uint8_t fail, const char * pre, char * num_char, size_t num_len, const char * post)
//{
//  const size_t bufferlen = 16;
//  char buffer[bufferlen];
//
//  size_t len = 0;
//  len = snprintf(buffer, bufferlen, "%-7s", pre);
//  HAL_UART_Transmit(MISC_HUART, (uint8_t *) buffer, len, 0xFFFFFFFF);
//  HAL_UART_Transmit(MISC_HUART, (uint8_t *) num_char, num_len, 0xFFFFFFFF);
//  len = snprintf(buffer, bufferlen, " %-4s|", post);
//
//  if (fail == 0)      len = snprintf(buffer, bufferlen, "\033[0;41m%-4s\033[0m", "FAIL\n");      // Red
//  else if (fail == 1) len = snprintf(buffer, bufferlen, "\033[0;42m%-7s\033[0m", "PASS\n"); // Green
//  else                len = snprintf(buffer, bufferlen, "\n", pre);                   // Nothing
//  HAL_UART_Transmit(MISC_HUART, (uint8_t *) buffer, len, 0xFFFFFFFF);
//
//  //HAL_UART_Transmit(MISC_HUART, (uint8_t *) num_char, num_len, 0xFFFFFFFF);
//
////  len = snprintf(buffer, bufferlen, " %-4s\033[0m|", post);
////  HAL_UART_Transmit(MISC_HUART, (uint8_t *) buffer, len, 0xFFFFFFFF);
//}

void misc_f32(float lo, float hi, float x, const char * pre, const char * number_format, const char * post)
{
  const size_t bufferlen = 16;
  char buffer[bufferlen];

  if (!verbose) return;

  uint8_t fail = 1;
  if (x < lo || x > hi) {
    fail = 0;
  } else if (isnan(lo) || isnan(hi)) {
    fail = 2;
  }
  size_t len = snprintf(buffer, bufferlen, number_format, x);
  misc_num(fail, pre, buffer, len, post);
}

void misc_i32(int32_t lo, int32_t hi, int32_t x, const char * pre, const char * number_format, const char * post)
{
  const size_t bufferlen = 16;
  char buffer[bufferlen];

  if (!verbose) return;

  uint8_t fail = 1;
  if (x < lo || x > hi) fail = 0;
  else if (isnan(lo) || isnan(hi)) fail = 2;

  size_t len = snprintf(buffer, bufferlen, number_format, x);
  misc_num(fail, pre, buffer, len, post);
}

void misc_u32(uint32_t lo, uint32_t hi, uint32_t x, const char * pre, const char * number_format, const char * post)
{
  const size_t bufferlen = 16;
  char buffer[bufferlen];

  if (!verbose) return;

  uint8_t fail = 1;
  if (x < lo || x > hi) fail = 0;
  else if (isnan(lo) || isnan(hi)) fail = 2;

  size_t len = snprintf(buffer, bufferlen, number_format, x);
  misc_num(fail, pre, buffer, len, post);
}

void misc_x16(uint16_t match, uint16_t x, const char * pre)
{
  const size_t bufferlen = 16;
  char buffer[bufferlen];
  const char number_format[] = "0x%04X";

  if (!verbose) return;
  uint8_t fail = 0;
  if (isnan(match)) fail = 2;
  else if (match == x) fail = 1;

  size_t len = snprintf(buffer, bufferlen, number_format, x);
  misc_num(fail, pre, buffer, len, " ");
}

size_t misc_getchar() // with echo
{
  uint8_t ch = 0;
  HAL_UART_Receive(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFF);
  HAL_UART_Transmit(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFF);
  return ch;
}

size_t misc_getline(uint8_t * line, size_t len)
{
  if (line == 0) {
    for (;;) {
      uint8_t ch = 0;
      HAL_UART_Receive(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFF);
      HAL_UART_Transmit(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFF);
      if (ch == '\n') return 0; // 0x0A
    }
  }

  uint16_t i;
  for (i = 0; i < len; i++) {
    HAL_UART_Receive(MISC_HUART, line + i, 1, 0xFFFF);
    if (line[i] == '\n') {
      line[i] = 0;  // drop \n and zero terminate
      return i + 1; // do not return '\n' 0x0A forLF, '\r' 0x0D for CR
    }
  }
  return len;
}

void misc_header(char * name, rosflight_firmware::PacketHeader &header)
{
  int64_t dt=0;
  if (header.timestamp>header.complete) dt = -(header.timestamp-header.complete);
  else dt = header.complete-header.timestamp;
  misc_printf("%-16s [t:%12.6f s dt:%10d us] ", name, (double) header.timestamp / 1e6, dt);
}

uint16_t misc_bytes_in_dma(DMA_HandleTypeDef * hdma_uart_rx, uint16_t dma_buffer_size)
{
  uint16_t size = dma_buffer_size - ((DMA_Stream_TypeDef *) hdma_uart_rx->Instance)->NDTR;
  return (size > dma_buffer_size) ? dma_buffer_size : size;
}

void misc_exit_status(uint32_t status)
{
  misc_printf("Exit Status: ");
  if (status == DRIVER_OK) misc_printf(" \033[0;42mDRIVER_OK");
  else misc_printf(" \033[0;41mDRIVER_ERROR");

  if (status & DRIVER_ID_MISMATCH) misc_printf(" DRIVER_ID_MISMATCH");
  if (status & DRIVER_SELF_DIAG_ERROR) misc_printf(" DRIVER_SELF_DIAG_ERROR");
  if (status & DRIVER_HAL_ERROR) misc_printf(" DRIVER_HAL_ERROR");
  if (status & DRIVER_HAL_ERROR2) misc_printf(" DRIVER_HAL_ERROR2");
  if (status & DRIVER_FIFO_INIT_ERROR) misc_printf(" DRIVER_FIFO_INIT_ERROR");
  if (status & UBX_ACK) misc_printf(" UBX_ACK,");
  if (status & UBX_NAK) misc_printf(" UBX_NAK,");
  if (status & UBX_ACKNAK_FAIL) misc_printf(" UBX_ACKNAK_FAIL");
  if (status & UBX_SUCCESS) misc_printf(" UBX_SUCCESS");
  if (status & UBX_FAIL_BAUD_CHANGE) misc_printf(" UBX_FAIL_BAUD_CHANGE");
  if (status & VOLTAGE_SET_FAIL) misc_printf(" VOLTAGE_SET_FAIL");
  misc_printf("\033[0m\n");
}

