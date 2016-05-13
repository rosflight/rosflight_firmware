#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "param.h"
#include "sensors.h"

uint32_t last_heartbeat_us;
uint32_t heartbeat_period_us;
uint32_t last_imu_us;
uint32_t imu_period_us;

void setup(void)
{
  i2cInit(I2CDEV_2);
  init_params();
  init_mavlink();

  last_heartbeat_us = 0;
  heartbeat_period_us = 1e6;
  last_imu_us = 0;
  imu_period_us = 10000;
}

void loop(void)
{
  uint32_t loop_time_us = micros();

  update_sensors(loop_time_us);

  if (loop_time_us - last_imu_us >= imu_period_us)
  {
      last_imu_us = loop_time_us;
      send_imu(1,2,3,4,5,6);
  }

  if (loop_time_us - last_heartbeat_us >= heartbeat_period_us)
  {
    last_heartbeat_us = loop_time_us;
    send_heartbeat();
  }
}
