#pragma once

#include <stdint.h>

/**
 * @brief Send a named integer debug value over mavlink
 * @param name The name for the value
 * @param value The value
 */
void mavlink_send_named_value_int(const char *const name, int32_t value);

/**
 * @brief Send a named float debug value over mavlink
 * @param name The name for the value
 * @param value The value
 */
void mavlink_send_named_value_float(const char *const name, float value);
