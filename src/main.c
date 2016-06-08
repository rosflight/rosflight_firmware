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
  init_mode();
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
    // loop time calculation
    dt = now - prev_time;
    prev_time = now;
    average_time+=dt;

    // If I have new IMU data, then perform control
    run_estimator(now);

    run_controller(now); // 6us

    mix_output(); // 1 us

    if(counter > 50)
    {
      printf("rc = %d\t%d\t%d\t%d\tcombined=%d\t%d\t%d\t%d\tout=%d\t%d\t%d\t%d\n",
             _rc_control.x.value,
             _rc_control.y.value,
             _rc_control.z.value,
             _rc_control.F.value,
             _combined_control.x.value,
             _combined_control.y.value,
             _combined_control.z.value,
             _combined_control.F.value,
             _outputs[0],
             _outputs[1],
             _outputs[2],
             _outputs[3]);
      counter = 0;
    }
    counter++;
  }

  if(counter > 10)
  {
//    printf("average time = %d us\n", average_time/counter);
//    printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//        _gyro_scale,
//        _current_state.p/1000,
//        _current_state.q/1000,
//        _current_state.r/1000,
//        _current_state.phi/1000,
//        _current_state.theta/1000,
//        _current_state.psi/1000);
//    counter = 0;
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
//  receive_rc(now); // 1 us
  _rc_control.x.active = true;
  _rc_control.y.active = true;
  _rc_control.z.active = true;
  _rc_control.F.active = true;
  _rc_control.x.type = PASSTHROUGH;
  _rc_control.y.type = PASSTHROUGH;
  _rc_control.z.type = PASSTHROUGH;
  _rc_control.F.type = THROTTLE;
  _rc_control.x.value = 500;
  _rc_control.y.value = 500;
  _rc_control.z.value = 500;
  _rc_control.F.value = 500;
  _new_command = true;

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_inputs(); // 3 us
}



