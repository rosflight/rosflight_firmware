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
#include "mixer.h"

void setup(void)
{
  // Load Default Params
  // Read EEPROM to get initial params
  // init_params();

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize I2c
//  i2cInit(I2CDEV_2);

  // Initialize PWM
  bool useCPPM = _params.values[PARAM_RC_TYPE];
  int16_t motor_refresh_rate = _params.values[PARAM_MOTOR_PWM_SEND_RATE];
  int16_t idle_pwm = _params.values[PARAM_IDLE_PWM];
  pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);


  // Initialize Serial Communication
//  init_mavlink();
//  init_sensors();


  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  init_mixing();
  // Initialize Estimator
//  init_estimator();

  delay(500);
  i2cInit(I2CDEV_2);
  init_sensors();
  init_estimator();
}

uint32_t counter = 0;
uint32_t average_time = 0;

void loop(void)
{
  /// Pre-process
  // get loop time
  static uint32_t prev_time;
  static int32_t dt = 0;
  uint32_t now = micros();
  dt = now - prev_time;
  prev_time = now;

  _command.F = 1500;
  _command.x = 1000;
  _command.y = 1000;

  pwmWriteMotor(0,1100);


  if(update_sensors(now));
    run_estimator(now);

//  if(counter > 10000){
//    printf("accx = %d accy = %d accz = %d gx = %d gy = %d gz = %d\n",
//           (_accel_data[0]*_accel_scale)/1000,
//           (_accel_data[1]*_accel_scale)/1000,
//           (_accel_data[2]*_accel_scale)/1000,
//           (_gyro_data[0]*_gyro_scale)/1000,
//           (_gyro_data[1]*_gyro_scale)/1000,
//           (_gyro_data[2]*_gyro_scale)/1000);
//    printf("phi = %d theta = %d psi = %d dt = %d \n\n", _current_state.phi, _current_state.theta, _current_state.psi, average_time/10000);
//    average_time = 0;
//    counter = 0;
//  }
  counter++;
  average_time += dt;

//  mix_output();
  delay(500);
}
