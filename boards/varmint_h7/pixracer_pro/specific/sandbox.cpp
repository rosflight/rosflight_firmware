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

#include "sandbox.h"

#include "BoardConfig.h"
#include "misc.h"

#include "Varmint.h"
extern Varmint varmint;

extern Time64 time64;

extern bool verbose;

#define ROWSIZE 256
#define ASCII_ESC 27

void verbose_dashes(void)
{
  for (int i = 0; i < ROWSIZE; i++) misc_printf("-");
  misc_printf("\n");
}

void verbose_equals(void)
{
  for (int i = 0; i < ROWSIZE; i++) misc_printf("=");
  misc_printf("\n");
}

void sandbox_dashboard(bool clear)
{
  verbose = true;

  if (clear) misc_printf("%c[2J", ASCII_ESC);

  misc_printf("%c[H", ASCII_ESC); // home

  verbose_equals();
  misc_printf("SANDBOX DASHBOARD PIXRACER_PRO\n");
  verbose_equals();

  varmint.imu0_.display();
  verbose_dashes();
  varmint.mag_.display();
  verbose_dashes();
  varmint.baro_.display();
  verbose_dashes();
  varmint.pitot_.display();
  verbose_dashes();
  varmint.adc_.display();
  verbose_dashes();
  varmint.rc_.display();
  verbose_dashes();
  varmint.gps_.display();

  verbose_equals();
}

void sandbox(void)
{
  // Give us time to read the initialization messages
  time64.dMs(5000);

  verbose = true;

  //	 Test pwm outputs
  //
  //	float rates[PWM_CHANNELS] = {3e5,3e5,3e5,3e5,50,50,490,490};
  //	//float rates[PWM_CHANNELS] = {50,50,50,50,50,50,50,50};
  //	varmint.pwm_.updateConfig(rates, PWM_CHANNELS);
  //	float outputs[PWM_CHANNELS] = { 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8};
  //	varmint.pwm_.write(outputs, PWM_CHANNELS);
  //
  //	while(1)
  //	{
  //	  PROBE1_HI;
  //	  varmint.pwm_.write(outputs, PWM_CHANNELS);
  //	  PROBE1_LO;
  //	  time64.dUs(450); ~ 2khs update rate
  //	}

  uint32_t n = 0;
  while (1) { sandbox_dashboard((n++) % 100 == 0); }
}
