#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "param.h"
#include "mode.h"
#include "sensors.h"
#include "estimator.h"
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
  init_estimator();

}

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

  mix_output();
  delay(500);
}
