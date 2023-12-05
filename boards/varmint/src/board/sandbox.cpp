/**
 ******************************************************************************
 * File     : sandbox.cpp
 * Date     : Sep 28, 2023
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
#include <Varmint.h>
#include <misc.h>
#include <sandbox.h>
extern Varmint varmint;

extern Time64 time64;

extern bool verbose;

#define ROWSIZE 180
#define ASCII_ESC 27

void verbose_dashes(void)
{
  for (int i = 0; i < ROWSIZE; i++) misc_printf("-");
  misc_printf("\n\r");
}

void verbose_equals(void)
{
  for (int i = 0; i < ROWSIZE; i++) misc_printf("=");
  misc_printf("\n\r");
}

void sandbox(void)
{
  time64.dMs(5000);

  //	while(1)
  //	{
  //		for(uint32_t t = 1; t<=100 ; t++)
  //		{
  //			PB1_HI;
  //			PH1_HI;
  //			time64.dMs(t);
  //			PB1_LO;
  //			PH1_LO;
  //			time64.dMs(t);
  //		}
  //	}

  //	{
  //		uint32_t n=2;
  //		varmint.pwm_[n].enable();
  //
  //		while(1)
  //		{
  //			PH1_LO;
  //			varmint.pwm_[n].writeUs(1000);
  //			time64.dMs(500);
  //			PH1_HI;
  //			varmint.pwm_[n].writeUs(2000);
  //			time64.dMs(500);
  //		}
  //	}

  verbose = true;
  uint32_t n = 0;
  while (1)
  {
    sandbox_dashboard((n++) % 100 == 0);
    time64.dMs(100);
  }
}

void sandbox_dashboard(bool clear)
{
  verbose = true;

  if (clear)
    misc_printf("%c[2J", ASCII_ESC);

  misc_printf("%c[H", ASCII_ESC); // home
  verbose_equals();
  misc_printf("SANDBOX DASHBOARD\n\r");
  verbose_dashes();
  varmint.imu0_.display();
  verbose_dashes();
  varmint.imu1_.display();
  verbose_dashes();
  varmint.mag_.display();
  verbose_dashes();
  varmint.pitot_.display();
  verbose_dashes();
  varmint.baro_.display();
  verbose_dashes();
  varmint.adc_.display();
  verbose_dashes();
  varmint.rc_.display();
  verbose_dashes();
  varmint.gps_.display();
  verbose_equals();
}
