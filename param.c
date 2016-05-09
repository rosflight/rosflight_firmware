#include "param.h"

// global variable definitions
params_t _params;

// function definitions
void init_params(void)
{
  _params.sensors.imu_period_us = 5000;

  _params.mavlink.system_id = 1;
  _params.mavlink.component_id = 250;
  _params.mavlink.stream_heartbeat = true;
  _params.mavlink.heartbeat_period_us = 1e6;
  _params.mavlink.stream_imu = true;
  _params.mavlink.imu_period_us = 5000;
}
