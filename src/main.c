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

void setup(void)
{
  delay(500);

  // Load Default Params
  // Read EEPROM to get initial params
  init_params();

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize I2c
  i2cInit(I2CDEV_2);

  // Initialize PWM
  bool useCPPM = _params.values[PARAM_RC_TYPE];
  int16_t motor_refresh_rate = _params.values[PARAM_MOTOR_PWM_SEND_RATE];
  int16_t idle_pwm = _params.values[PARAM_IDLE_PWM];
  pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);


  // Initialize Serial Communication
  init_mavlink();
  init_sensors();


  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  // Initialize Estimator
  init_estimator();

  // Initialize Controller
  // Initialize State Machine
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

  // update sensors (only the ones that need updating)
  // If I have new IMU data, then perform control, otherwise, do something else
  if (update_sensors(now))
  {
    // run estimator
    run_estimator(now);
    printf("phi = %d theta = %d psi = %d\n", _current_state.phi/1000, _current_state.theta/1000, _current_state.psi/1000);

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
  }
  counter++;
  average_time += dt;

  // send serial sensor data
  // send low priority messages (e.g. param values)
  //  internal timers figure out what to send
//  mavlink_stream(now);

  /// Post-Process
  // receive mavlink messages
//  mavlink_receive();

  // commands from the computer will be updated by callbacks
  // update controlModeComp

  // (for next steps get most recent RC value from drv_pwm as needed)
  // if it has been at least 50 Hz
  // update overrideMode (read switch if present)
  // OFFBOARD               (can override RPY by moving RC out of deadzone)
  // OFFBOARD_MIN_THROTTLE  (same as above, but takes min throttle)
  // MANUAL_RC              (listens only to RC)
  // RC switch moves between MANUAL_RC and OFFBOARD_xx
  // which offboard mode you go to is set by a param
  // update controlModeRC (read switch if present)
  // update armedState (read switch)
}
