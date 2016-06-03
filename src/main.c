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
#include "rc.h"

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
  bool useCPPM = false;//_params.values[PARAM_RC_TYPE];
  int16_t motor_refresh_rate = _params.values[PARAM_MOTOR_PWM_SEND_RATE];
  int16_t idle_pwm = _params.values[PARAM_IDLE_PWM];
  pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);
  init_rc();


  // Initialize Serial Communication
//  init_mavlink();
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
  }
  average_time += dt;

  // send serial sensor data
  // send low priority messages (e.g. param values)
  //  internal timers figure out what to send
//  mavlink_stream(now);

  /// Post-Process
  // receive mavlink messages
//  mavlink_receive();

  // receive rc (if time)
  check_mode(now);
  if( receive_rc(now))
  {
    if(counter > 10)
    {
    printf("\nNew RC:\n");
    printf("rc: %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", pwmRead(0), pwmRead(1), pwmRead(2), pwmRead(3), pwmRead(4), pwmRead(5), pwmRead(6), pwmRead(7));
    printf("flags:  %d\t%d\t%d\t%d\n", _rc_control.x.active, _rc_control.y.active, _rc_control.z.active, _rc_control.F.active );
    printf("Types:  %d\t%d\t%d\t%d\n", _rc_control.x.type, _rc_control.y.type, _rc_control.z.type, _rc_control.F.type );
    printf("Values: %d\t%d\t%d\t%d\n", _rc_control.x.value, _rc_control.y.value, _rc_control.z.value, _rc_control.F.value );
    printf("Mode:   %d\t%d\n", _armed_state,pwmRead(_params.values[PARAM_RC_Z_CHANNEL]) < (_params.values[PARAM_RC_Z_CENTER]-_params.values[PARAM_RC_Z_RANGE]/2) + _params.values[PARAM_ARM_THRESHOLD]);
    counter = 0;
    }
    else
    {
      counter++;
    }
  }

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

