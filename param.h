#pragma once

#include <stdbool.h>
#include <stdint.h>

// type definitions
typedef struct
{
  uint32_t imu_period_us;
} sensorParams_t;

typedef struct
{
  uint8_t system_id;
  uint8_t component_id;

  bool stream_heartbeat;
  uint32_t heartbeat_period_us;

  bool stream_imu;
  uint32_t imu_period_us;
} mavlinkParams_t;

typedef struct
{
  sensorParams_t sensors;
  mavlinkParams_t mavlink;
} params_t;

// global variable declarations
extern params_t _params;

// function declarations
void init_params(void);
