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

#include <BoardConfig.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <usb_device.h>
#include <usbd_cdc_acm_if.h>

#include <misc.h>

extern bool verbose;

extern "C" {
int __io_putchar(int ch)
{
#ifdef __HAVE_SWO__
  // remove '\r'
  if (ch != '\r') {
    ITM_SendChar(ch); // for SWO
  }
  return 1;
#else
  HAL_UART_Transmit(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFFFFFF);
#endif
  return 1;
}

int __io_getchar(void)
{
  uint8_t ch = 0;
  HAL_UART_Receive(MISC_HUART, (uint8_t *) (&ch), 1, 0xFFFF);
  // ch = ITM_ReceiveChar(); // for SWO, not supported on STM32CubeIDE.
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
// #define MAX_SPRINTF_CHARS 256
// DMA_RAM char misc_sprintf_buffer[MAX_SPRINTF_CHARS];
void misc_printf(const char * format, ...)
{
  if (!verbose) return;

  va_list argp;
  va_start(argp, format);
  vprintf(format, argp);
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

void misc_header(char * name, uint64_t drdy, uint64_t timestamp, uint64_t delay)
{
  misc_printf("%-16s [%8.2f s %8.2f ms %8.3f ms] ", name, (double) drdy / 1e6, (double) (timestamp - drdy) / 1000.,
              (double) delay / 1000.);
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

  if (status != DRIVER_OK && verbose) {
    while (1)
      ;
  } //PTT Need to inform error handler.
}
