#include <string.h>

#include <breezystm32/breezystm32.h>

#include "flash.h"

void initEEPROM(void)
{
  // make sure (at compile time) that config struct doesn't overflow allocated flash pages
  // ct_assert(sizeof(_params) < CONFIG_SIZE);
}

static uint8_t compute_checksum(const void * addr, size_t len)
{
  const uint8_t *p;
  uint8_t chk = 0;

  for (p = (const uint8_t *)addr; p < ((const uint8_t *)addr + len); p++)
    chk ^= *p;

  return chk;
}

bool readEEPROM(void * dest, size_t len)
{
  memcpy(dest, (char *)FLASH_WRITE_ADDR, len);
  return true;
}

bool writeEEPROM(const void * src, size_t len)
{
  FLASH_Status status;

  // write it
  FLASH_Unlock();
  for (unsigned int tries = 3; tries; tries--)
  {
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for(int i = 0; i < NUM_PAGES; i++)
      status = FLASH_ErasePage(FLASH_WRITE_ADDR + i * FLASH_PAGE_SIZE);
    for (unsigned int i = 0; i < len && status == FLASH_COMPLETE; i += 4)
    {
      status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)((char *)src + i));
      if (status != FLASH_COMPLETE)
      {
        continue;
      }
    }
    if(status == FLASH_COMPLETE)
      break;
  }
  FLASH_Lock();

  // Flash write failed - just die now
  if (status != FLASH_COMPLETE
    || compute_checksum(src, len) != compute_checksum(FLASH_WRITE_ADDR, len))
  {
    return false;
  }

  return true;
}
