#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "param.h"
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
