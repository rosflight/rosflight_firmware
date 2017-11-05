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

#include "revo.h"
#include "rosflight.h"
//#include "mavlink.h"

extern "C"
{
void WWDG_IRQHandler(void)
{
  volatile int debug = 1;
}

void UsageFault_Handler(void)
{
  volatile int debug =1 ;
}

void USART6_IRQHandler(void)
{
  volatile int debug = 1;
}

void USART3_IRQHandler(void)
{
  volatile int debug = 1;
}

void USART2_IRQHandler(void)
{
  volatile int debug = 1;
}

void USART1_IRQHandler(void)
{
  volatile int debug = 1;
}

void UART5_IRQHandler(void)
{
  volatile int debug = 1;
}

void UART4_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM8_UP_TIM13_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  volatile int debug = 1;
}


void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM8_CC_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM1_CC_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
  volatile int debug = 1;
}

void TIM7_IRQHandler()
{
  volatile int debug = 1;
}

void TIM5_IRQHandler()
{
  volatile int debug = 1;
}

void TIM4_IRQHandler()
{
  volatile int debug = 1;
}

void TIM2_IRQHandler()
{
  volatile int debug = 1;
}

void TIM6_DAC_IRQHandler()
{
  volatile int debug = 1;
}

void TAMP_STAMP_IRQHandler()
{
  volatile int debug = 1;
}

void SPI3_IRQHandler(){}
void SPI2_IRQHandler(){}
void SPI1_IRQHandler(){}
void SDIO_IRQHandler(){}
void RTC_WKUP_IRQHandler(){}
void RTC_Alarm_IRQHandler(){}
void RCC_IRQHandler(){}
void PVD_IRQHandler(){}
void OTG_HS_WKUP_IRQHandler(){}
void OTG_HS_IRQHandler(){}
void OTG_HS_EP1_OUT_IRQHandler(){}
void OTG_HS_EP1_IN_IRQHandler(){}
void MemManage_Handler(){}
void I2C3_EV_IRQHandler(){}
void I2C3_ER_IRQHandler(){}
void HardFault_Handler(){}
}

int main(void)
{
  rosflight_firmware::Revo board;
  board.init_board();

  int i = 0;
  while (i < 10)
  {
    board.clock_delay(10);
    i += 1;
  }

  rosflight_firmware::ROSflight firmware(board);

//  firmware.init();

  while(1)
  {
//    firmware.run();
    board.led0_toggle();
    board.clock_delay(100);
  }
  return 0;
}

