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
//    init_mavlink();

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
  _armed_state = ARMED;
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

    mix_output();
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
  // internal timers figure out what to send
//    mavlink_stream(now);

  // receive mavlink messages
//    mavlink_receive();

  // update the armed_states, an internal timer runs this at a fixed rate
//  check_mode(now); // 0 us

  // get RC, an internal timer runs this every 20 ms (50 Hz)
//  receive_rc(now); // 1 us
  _rc_control.x.active = true;
  _rc_control.y.active = true;
  _rc_control.z.active = true;
  _rc_control.F.active = true;
  _rc_control.x.type = ANGLE;
  _rc_control.y.type = ANGLE;
  _rc_control.z.type = RATE;
  _rc_control.F.type = THROTTLE;
  _rc_control.x.value = 10;
  _rc_control.y.value = 10;
  _rc_control.z.value = 0;
  _rc_control.F.value = 100;
  _new_command = true;

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_inputs(); // 3 us

  if(counter > 100)
  {
    printf("average time = %d us\n", average_time/counter);
    printf("\n\nrc\t%d\t%d\t%d\t%d\n",
           _rc_control.x.value,
           _rc_control.y.value,
           _rc_control.z.value,
           _rc_control.F.value);
    printf("combined %d\t%d\t%d\t%d\n",
        _combined_control.x.value,
        _combined_control.y.value,
        _combined_control.z.value,
        _combined_control.F.value);
    printf("command   %d\t%d\t%d\t%d\n",
        _command.x,
        _command.y,
        _command.z,
        _command.F);
    mix_output(); // 1 us
    printf("output   %d\t%d\t%d\t%d\n",
         _outputs[0],
        _outputs[1],
        _outputs[2],
        _outputs[3]);
    printf("state %d\t%d\t%d\n",
           _current_state.phi/1000,
           _current_state.theta/1000,
           _current_state.psi/1000);


    counter = 0;
    average_time = 0;
  }
  counter++;


}



