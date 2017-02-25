#include "board.h"

#include "flash.h"

#include <breezystm32/breezystm32.h>

extern void SetSysClock(bool overclock);
serialPort_t *Serial1;

void init_board(void)
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();

  // Initialize Serial ports
  Serial1 = uartOpen(USART1, NULL, get_param_int(PARAM_BAUD_RATE), MODE_RXTX);
}

// serial

void serial_write(uint8_t byte)
{
  serialWrite(Serial1, byte);
}

uint16_t serial_bytes_available(void)
{
  return serialTotalBytesWaiting(Serial1);
}

uint8_t serial_read(void)
{
  return serialRead(Serial1);
}

// non-volatile memory

void memory_init(void)
{
  initEEPROM();
}

bool memory_read(void * dest, size_t len)
{
  return readEEPROM(dest, len);
}

bool memory_write(const void * src, size_t len)
{
  return writeEEPROM(src, len);
}
