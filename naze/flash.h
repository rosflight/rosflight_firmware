#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <breezystm32/breezystm32.h>

// #define ASSERT_CONCAT_(a, b) a##b
// #define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
// #define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

// define this symbol to increase or decrease flash size. not rely on flash_size_register.
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif

#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define NUM_PAGES                       3
// if sizeof(_params) is over this number, compile-time error will occur. so, need to add another page to config data.
// TODO compile time check is currently disabled
#define CONFIG_SIZE                     (FLASH_PAGE_SIZE * NUM_PAGES)

// static const uint8_t EEPROM_CONF_VERSION = 76;
//static uint32_t enabledSensors = 0;
//static void resetConf(void);
static const uint32_t FLASH_WRITE_ADDR = 0x08000000 + (FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - (CONFIG_SIZE / 1024)));


/**
 * @brief Initialize Flash
 */
void initEEPROM(void);

/**
 * @brief Read data from Flash
 * @param dest The memory address to copy the data to
 * @param len The number of bytes to copy
 * @returns true if the read was successful, false otherwise
 */
bool readEEPROM(void * dest, size_t len);

/**
 * @brief Write data to Flash
 * @param src The memory address to copy data from
 * @param len The number of bytes to copy
 * @returns true if the write was successful, false otherwise
 */
bool writeEEPROM(const void * src, size_t len);
