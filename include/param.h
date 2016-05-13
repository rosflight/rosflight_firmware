#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mavlink_bridge.h"

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
void init_params(void);
uint8_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);
bool set_param_by_id(uint8_t id, int32_t value);
bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value);
