#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"
#include "mavlink.h"
#include "mavlink_stream.h"

#include "param.h"

//TODO temporary
#include <stdio.h>

// global variable definitions
params_t _params;

// local function definitions
static void init_param(paramId_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  _params.values[id] = value;
  strcpy(_params.names[id], name);
}

// function definitions
void init_params(void)
{
  initEEPROM();
  if(!readEEPROM())
  {
    init_param(PARAM_SYSTEM_ID, "SYS_ID", 1);
    init_param(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1);
    init_param(PARAM_STREAM_IMU_RATE, "STRM_IMU", 100);

    // temporary: replace with actual initialisation of rest of params
    char temp_name[PARAMS_NAME_LENGTH];
    for (paramId_t id = 3; id < PARAMS_COUNT; id++)
    {
      sprintf(temp_name, "TEMP%d", id);
      init_param(id, temp_name, id);
    }

    for (paramId_t id = 0; id < PARAMS_COUNT; id++)
      param_change_callback(id);

    writeEEPROM(true);
  }
}

void param_change_callback(paramId_t id)
{
  switch (id)
  {
  case PARAM_SYSTEM_ID:
    mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
    break;
  case PARAM_STREAM_HEARTBEAT_RATE:
    mavlink_stream_set_heartbeat_period_us(_params.values[PARAM_STREAM_HEARTBEAT_RATE] == 0 ?
                                             0 : 1e6 / _params.values[PARAM_STREAM_HEARTBEAT_RATE]);
    break;
  case PARAM_STREAM_IMU_RATE:
    mavlink_stream_set_imu_period_us(_params.values[PARAM_STREAM_IMU_RATE] == 0 ?
                                       0 : 1e6 / _params.values[PARAM_STREAM_IMU_RATE]);
    break;
  default:
    // no action needed for this parameter
    break;
  }
}

paramId_t lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (paramId_t id = 0; id < PARAMS_COUNT; id++)
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
