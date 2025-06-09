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

#define ROWSIZE 180
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
  misc_printf("SANDBOX DASHBOARD VARMINT_10X\n");
  verbose_equals();

  varmint.imu0_.display();
  verbose_dashes();
  varmint.imu1_.display();
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

  //  time64.dMs(5000);
  //  SD Card read/write (and CRC & RNG)
  //  #define RNGLEN (512)
  //  static uint32_t random_numbers[512];
  //  for(int i=0;i<RNGLEN;i++) HAL_RNG_GenerateRandomNumber(&hrng, &(random_numbers[i]));
  //
  //  uint16_t len = RNGLEN*sizeof(uint32_t)-3; // subtract 1 to make it odd.
  //  uint8_t *test1 = (uint8_t *)random_numbers;
  //
  //  // Write the test1 data to SD
  //  bool ok1 = varmint.sd_.write(test1,len);
  //  if(ok1) misc_printf("OK1 \n"); else misc_printf("NOT OK1 \n");
  //
  //  // result array
  //  static uint8_t test2[RNGLEN*sizeof(uint32_t)];
  //
  //  // Read the data back
  //
  //  bool ok2 = varmint.sd_.read(test2,len);
  //  if(ok2) misc_printf("OK2 \n");  else misc_printf("NOT OK2 \n");
  //
  //  for(int i=1; i<len; i++)
  //  {
  //    if (test1[i]!=test2[i]) misc_printf("[%u] %02X -> %02X\n ",i,test1[i],test2[i]);
  //  }
  //
  //  time64.dMs(5000);
  //	 Test pwm outputs
  //
  //	float rates[PWM_CHANNELS] = {3e5,3e5,3e5,3e5,6e5,6e5,6e5,6e5,490,490};
  //	//float rates[PWM_CHANNELS] = {50,50,50,50,50,50,50,50,50,50,50,50};
  //	varmint.pwm_.updateConfig(rates, PWM_CHANNELS);
  //	float outputs[PWM_CHANNELS] = { 0.0, 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8, 1.0};
  //	varmint.pwm_.write(outputs, PWM_CHANNELS);
  //
  //	while(1)
  //	{
  //	  PROBE1_HI;
  //	  varmint.pwm_.write(outputs, PWM_CHANNELS);
  //	  PROBE1_LO;
  //	  time64.dUs(450); ~ 2khs update rate
  //	}

  time64.dMs(5000);

  verbose = true;
  uint32_t n = 0;
  while (1) {
    sandbox_dashboard((n++) % 100 == 0);
    // time64.dMs(200);
  }
}
