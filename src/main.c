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
  last_imu_us = 0;
}

void loop(void)
{
  uint32_t loop_time_us = micros();

  update_sensors(loop_time_us);

  if (loop_time_us - last_imu_us >= _params.values[PARAM_STREAM_IMU_RATE])
  {
    last_imu_us = loop_time_us;
    send_imu(1,2,3,4,5,6);
  }

  if (loop_time_us - last_heartbeat_us >= _params.values[PARAM_STREAM_HEARTBEAT_RATE])
  {
    last_heartbeat_us = loop_time_us;
    send_heartbeat();
  }

  mavlink_receive();
  mavlink_send_low_priority();
}
