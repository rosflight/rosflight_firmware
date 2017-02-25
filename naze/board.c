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

// PWM

void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t pwm_read(uint8_t channel);
{
  return pwmRead(channel);
}

void pwm_write(uint8_t channel, uint16_t value)
{
  pwmWriteMotor(channel, value);
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

// LED

void led0_on(void) { LED0_ON; }
void led0_off(void) { LED0_OFF; }
void led0_toggle(void) { LED0_TOGGLE; }

void led1_on(void) { LED1_ON; }
void led1_off(void) { LED1_OFF; }
void led1_toggle(void) { LED1_TOGGLE; }
