#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32/breezystm32.h>

#include "estimator.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "mode.h"
#include "param.h"
#include "sensors.h"
#include "controller.h"
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
  //  init_mavlink();

  // Initialize Sensors
  init_sensors();


  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  init_mixing();

  // Initialize Estimator
  init_estimator();
  //  init_mode();
}

uint32_t counter = 0;
uint32_t average_time = 0;

void loop(void)
{
  /*********************/
  /***  Pre-Process ***/
  /*********************/
  // get loop time
  static uint32_t prev_time;
  static int32_t dt = 0;
  uint32_t now = micros();

  /*********************/
  /***  Control Loop ***/
  /*********************/
  // update sensors - an internal timer runs this at a fixed rate
  if (update_sensors(now)) // 434 us
  {
    // If I have new IMU data, then perform control
    run_estimator(now);

    _combined_control.x.type = ANGLE;
    _combined_control.y.type = ANGLE;
    _combined_control.z.type = RATE;
    _combined_control.F.type = THROTTLE;

    _combined_control.x.active = true;
    _combined_control.y.active = true;
    _combined_control.z.active = true;
    _combined_control.F.active = true;

    _combined_control.F.value = 500;
    _combined_control.x.value = 0;
    _combined_control.y.value = 0;
    _combined_control.z.value = 0;

    run_controller(now); // 6us

    // loop time calculation
    dt = now - prev_time;
    prev_time = now;
    average_time+=dt;
    counter++;

    mix_output(); // 1 us

    if(counter == 50)
    {
//      printf("outputs:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//             _outputs[0],
//             _outputs[1],
//             _outputs[2],
//             _outputs[3],
//             _outputs[4],
//             _outputs[5],
//             _outputs[6],
//             _outputs[7]);
    }
  }

  if(counter > 1000)
  {
    printf("average time = %d us\n", average_time/counter);
    counter = 0;
    average_time = 0;
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
  // internal timers figure out what to send
  //  mavlink_stream(now);

  // receive mavlink messages
  //  mavlink_receive();

  // update the armed_states, an internal timer runs this at a fixed rate
  check_mode(now); // 0 us

  // get RC, an internal timer runs this every 20 ms (50 Hz)
  receive_rc(now); // 1 us

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_inputs(); // 3 us
}


