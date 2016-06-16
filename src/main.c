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
  init_mixing();

  // Initialize Estimator
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

  // update sensors (only the ones that need updating)
  // If I have new IMU data, then perform control, otherwise, do something else
  if (update_sensors(now))
  {
    // run estimator
    run_estimator(now);

    /// Need to mix between RC and computer based on override mode and RC & computer control modes
    /// (happens at multiple levels between control loops)
    switch (armed_state)
    {
    case ARMED:
      switch (composite_control_mode)
      {
      case ALT_MODE:
        // thrust_c = runAltController(alt_c, state);
      case ATTITUDE_MODE:
        // omega_c = runAttController(theta_c, state);
      case RATE_MODE:
        // tau_c = runRateController(omega_c, state);
        // motor_speeds = mixOutput(tau_c);
      case PASSTHROUGH:
        // write_motors_armed(motor_speeds);
        break;
      default:
        error_state = INVALID_CONTROL_MODE;
        break;
        break;
      }
    case DISARMED:
      // write_motors_armed();
      break;
    default:
      error_state = INVALID_ARMED_STATE;
    }
  }
  counter++;
  average_time += dt;

  // send serial sensor data
  // send low priority messages (e.g. param values)
  //  internal timers figure out what to send
  mavlink_stream(now);

  /// Post-Process
  // receive mavlink messages
  mavlink_receive();
}
