#include <breezystm32/breezystm32.h>

#include "param.h"

#include "flash.h"

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

bool writeEEPROM(bool blink)
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

    FLASH_ErasePage(FLASH_WRITE_ADDR);
    status = FLASH_ErasePage(FLASH_WRITE_ADDR + FLASH_PAGE_SIZE);
    for (unsigned int i = 0; i < sizeof(params_t) && status == FLASH_COMPLETE; i += 4)
      status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)((char *)&_params + i));
    if (status == FLASH_COMPLETE)
      break;
  }
  FLASH_Lock();

  // Flash write failed - just die now
  if (status != FLASH_COMPLETE || !validEEPROM())
  {
    return false;
  }

  if (blink)
  {
    for (uint8_t i=0; i < 3; i++)
    {
      LED0_TOGGLE;
      delay(100);
      LED0_TOGGLE;
      delay(50);
    }
  }

  return true;
}
