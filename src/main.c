#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_pwm.h>

#include <turbotrig/turbotrig.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "param.h"
#include "mode.h"
#include "sensors.h"
#include "estimator.h"

void setup(void)
{
  i2cInit(I2CDEV_2);
  init_sensors();
  init_estimator();
}

void loop(void)
{
  static int32_t average_time;
  static counter =0;
  static int32_t dt = 0;
  static uint32_t prev_time = 0;
  uint32_t now = micros();
  dt = now - prev_time;
  prev_time = now;

  update_sensors(now);
  run_estimator(dt);

  if(counter > 10000){
    printf("phi = %d theta = %d psi = %d dt = %d \n", _current_state.phi, _current_state.theta, _current_state.psi, average_time/10000);
    average_time = 0;
    counter = 0;
  }
  counter++;
  average_time += dt;

}
