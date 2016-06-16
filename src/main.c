#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32/breezystm32.h>

#include "estimator.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "mavlink_util.h"
#include "mode.h"
#include "param.h"
#include "sensors.h"
#include "mixer.h"
#include "rc.h"

void setup(void)
{
  // Make sure all the perhipherals are done booting up before starting
  delay(500);

  // Load Default Params
  // Read EEPROM to get initial params
  init_params();

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize I2c
  i2cInit(I2CDEV_2);

  // Initialize PWM and RC
  init_PWM();
  init_rc();

  // Initialize MAVlink Communication
  init_mavlink();

  // Initialize Sensors
  init_sensors();


  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  init_mixing();

  // Initialize Estimator
  init_estimator();
  init_mode();
}

void loop(void)
{
  /*********************/
  /***  Pre-Process ***/
  /*********************/
  // get loop time
  static uint32_t prev_time;
  static int32_t dt = 0;
  uint32_t now = micros();
  dt = now - prev_time;
  prev_time = now;

  /*********************/
  /***  Control Loop ***/
  /*********************/
  // update sensors - an internal timer runs this at a fixed rate
  if (update_sensors(now))
  {
    // If I have new IMU data, then perform control
    // run estimator
    run_estimator(now);
//    run_controller(now);
//    mix_outputs();
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
  // internal timers figure out what to send
  mavlink_stream(now);

  // receive mavlink messages
  mavlink_receive();

  // update the armed_states, an internal timer runs this at a fixed rate
  check_mode(now);

  // get RC, an internal timer runs this every 20 ms (50 Hz)
  receive_rc(now);

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_inputs();
}

