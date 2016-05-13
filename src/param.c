#include <stdbool.h>
#include <stdint.h>

#include "param.h"

// global variable definitions
params_t _params;

// local function definitions
void init_param(uint8_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  _params.values[id] = value;
  strcpy(_params.names[id], name);
}

// function definitions
void init_params(void)
{
  init_param(PARAM_SYSTEM_ID, "SYS_ID", 1);
  init_param(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1000000);
  init_param(PARAM_STREAM_IMU_RATE, "STRM_IMU", 10000);
}

uint8_t lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (uint8_t id = 0; id < PARAMS_COUNT; id++)
  {
    bool match = true;
    for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++)
    {
      // compare each character
      if (name[i] != _params.names[id][i])
      {
        match = false;
        break;
      }

      // stop comparing if end of string is reached
      if (_params.names[id][i] == '\0')
        break;
    }

    if (match)
      return id;
  }

  return PARAMS_COUNT;
}

bool set_param_by_id(uint8_t id, int32_t value)
{
  if (id < PARAMS_COUNT)
  {
    _params.values[id] = value;
    return true;
  }

  return false;
}

bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  uint8_t id = lookup_param_id(name);
  if (id < PARAMS_COUNT)
  {
    _params.values[id] = value;
    return true;
  }

  return false;
}
