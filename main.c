#include <breezystm32.h>

#include "param.h"
#include "mavlink.h"
#include "sensors.h"

void setup(void)
{
  i2cInit(I2CDEV_2);
  init_params();
}

void loop(void)
{
  update_sensors(micros());
}
