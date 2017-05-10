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

#include <breezystm32/breezystm32.h>

#include "param.h"

#include "flash.h"
#include "mavlink_log.h"

void initEEPROM(void)
{
  // make sure (at compile time) that config struct doesn't overflow allocated flash pages
  ct_assert(sizeof(_params) < CONFIG_SIZE);
}

static bool validEEPROM(void)
{
  const params_t *temp = (const params_t *)FLASH_WRITE_ADDR;
  const uint8_t *p;
  uint8_t chk = 0;

  // check version number
  if (EEPROM_CONF_VERSION != temp->version)
    return false;

  // check size and magic numbers
  if (temp->size != sizeof(params_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
    return false;

  // verify integrity of temporary copy
  for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(params_t)); p++)
    chk ^= *p;

  // checksum failed
  if (chk != 0)
    return false;

  // looks good, let's roll!
  return true;
}

bool readEEPROM(void)
{
  // Sanity check
  if (!validEEPROM())
  {
    return false;
  }

  // Read flash
  memcpy(&_params, (char *)FLASH_WRITE_ADDR, sizeof(params_t));
  return true;
}

bool writeEEPROM()
{
  FLASH_Status status;
  uint8_t chk = 0;
  const uint8_t *p;

  // prepare checksum/version constants
  _params.version = EEPROM_CONF_VERSION;
  _params.size = sizeof(params_t);
  _params.magic_be = 0xBE;
  _params.magic_ef = 0xEF;
  _params.chk = 0;

  // recalculate checksum before writing
  for (p = (const uint8_t *)&_params; p < ((const uint8_t *)&_params + sizeof(params_t)); p++)
    chk ^= *p;
  _params.chk = chk;

  // write it
  FLASH_Unlock();
  for (unsigned int tries = 3; tries; tries--)
  {
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for (int i = 0; i < NUM_PAGES; i++)
      status = FLASH_ErasePage(FLASH_WRITE_ADDR + i * FLASH_PAGE_SIZE);
    for (unsigned int i = 0; i < sizeof(params_t) && status == FLASH_COMPLETE; i += 4)
    {
      status |= FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)((char *)&_params + i));
      if (status != FLASH_COMPLETE)
      {
        continue;
      }
    }
    if (status == FLASH_COMPLETE)
      break;
  }
  FLASH_Lock();

  // Flash write failed - just die now
  if (status != FLASH_COMPLETE || !validEEPROM())
  {
    return false;
  }

  return true;
}
