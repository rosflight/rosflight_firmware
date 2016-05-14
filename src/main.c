#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "param.h"
#include "sensors.h"

void setup(void)
{
  i2cInit(I2CDEV_2);
  init_params();
  init_mavlink();
}

void loop(void)
{
  uint32_t loop_time_us = micros();

  update_sensors(loop_time_us);

  mavlink_stream(loop_time_us);
  mavlink_receive();
}
