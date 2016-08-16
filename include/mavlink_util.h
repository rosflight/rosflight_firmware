#pragma once

#include <stdint.h>

#include "mux.h"

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

/**
 * @brief Send a named float debug value over mavlink
 * @param name The name for the struct
 * @param command_struct The command_struct
 */
void mavlink_send_named_command_struct(const char *const name, control_t command_struct);


