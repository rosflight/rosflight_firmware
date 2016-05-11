#include <breezystm32.h>

#include "mavlink.h"
#include "param.h"
#include "sensors.h"

uint32_t last_heartbeat_us;
uint32_t heartbeat_period_us;

void setup(void)
{
  i2cInit(I2CDEV_2);
  init_params();

  last_heartbeat_us = 0;
  heartbeat_period_us = 1e6;
}

void loop(void)
{
  uint32_t loop_time_us = micros();

  update_sensors(micros());
  if (loop_time_us - last_heartbeat_us >= heartbeat_period_us)
  {
    send_heartbeat();
  }
}
