#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mavlink.h"

#define PARAMS_COUNT 3
#define PARAMS_NAME_LENGTH MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN

#define PARAM_SYSTEM_ID 0
#define PARAM_STREAM_HEARTBEAT_RATE 1
#define PARAM_STREAM_IMU_RATE 2

// type definitions
typedef struct
{
  int32_t values[PARAMS_COUNT];
  char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
} params_t;

// global variable declarations
extern params_t _params;

// function declarations

/**
 * @brief Initialize parameter values
 */
void init_params(void);

/**
 * @brief Callback for executing actions that need to be taken when a parameter value changes
 * @param id The ID of the parameter that was changed
 */
void param_change_callback(uint8_t id);

/**
 * @brief Gets the id of a parameter from its name
 * @param name The name of the parameter
 * @returns The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
 */
uint8_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

/**
 * @brief Sets the value of a parameter by ID and calls the parameter change callback
 * @param id The ID of the parameter
 * @param value The new value
 * @returns True if a parameter value was changed, false otherwise
 */
inline bool set_param_by_id(uint8_t id, int32_t value)
{
  if (id < PARAMS_COUNT && value != _params.values[id])
  {
    _params.values[id] = value;
    param_change_callback(id);
    return true;
  }
  return false;
}

/**
 * @brief Sets the value of a parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @returns True if a parameter value was changed, false otherwise
 */
inline bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  uint8_t id = lookup_param_id(name);
  return set_param_by_id(id, value);
}
