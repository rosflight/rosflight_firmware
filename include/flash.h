#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

// define this symbol to increase or decrease flash size. not rely on flash_size_register.
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
// if sizeof(_params) is over this number, compile-time error will occur. so, need to add another page to config data.
#define CONFIG_SIZE                     (FLASH_PAGE_SIZE * 2)

static const uint8_t EEPROM_CONF_VERSION = 76;
//static uint32_t enabledSensors = 0;
//static void resetConf(void);
static const uint32_t FLASH_WRITE_ADDR = 0x08000000 + (FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - (CONFIG_SIZE / 1024)));


/**
 * @brief Initialize Flash
 */
void initEEPROM(void);

/**
 * @brief Read the _param struct from Flash
 * @returns true if the read was successful (validEEPROM), false otherwise
 */
bool readEEPROM(void);

/**
 * @brief write the _param struct to Flash
 * @param blink Blink the led after writing if true
 */
bool writeEEPROM(bool blink);
