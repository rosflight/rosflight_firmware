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
  //  init_PWM();
  //  init_rc();

  // Initialize MAVlink Communication
  //  init_mavlink();

  // Initialize Sensors
  init_sensors();


  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  //  init_mixing();

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
  if (update_sensors(now))
  {
    // If I have new IMU data, then perform control
    run_estimator(now);

    control_t rate_command;
    rate_command.F.value = 0;
    rate_command.x.value = 0;
    rate_command.y.value = 0;
    rate_command.z.value = 0;
    rate_command.x.type = rate_command.y.type = rate_command.z.type = ANGLE;
    rate_command.x.active = rate_command.y.active = rate_command.z.active = true;
    rate_command.F.type = THROTTLE;
    rate_command.F.active = true;

    control_t motor_command;

    dt = now - prev_time;
    prev_time = now;
    average_time+=dt;
    counter++;
    motor_command = attitude_controller(rate_command, now);
    //      printf("\ncontrol\n");
    //      printf("%d\t%d\t%d\t%d\n", motor_command.F.value, motor_command.x.value, motor_command.y.value, motor_command.z.value);
    //      printf("%d\t%d\t%d\t%d\n", motor_command.F.type, motor_command.x.type, motor_command.y.type, motor_command.z.type);
    //      printf("%d\t%d\t%d\t%d\n", motor_command.F.active, motor_command.x.active, motor_command.y.active, motor_command.z.active);
  }

  //    mix_outputs();

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
  check_mode(now);

  // get RC, an internal timer runs this every 20 ms (50 Hz)
  receive_rc(now);

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_inputs();
}


