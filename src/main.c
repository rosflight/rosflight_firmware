#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32/breezystm32.h>
#include <breezystm32/drv_pwm.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "param.h"
#include "mode.h"
#include "sensors.h"
#include "estimator.h"

void setup(void)
{
  // Load Default Params
  // Read EEPROM to get initial params
  // init_params();

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
  update_sensors(now);

  /// Main Thread
  // run estimator
  run_estimator(dt);

    /// Need to mix between RC and computer based on override mode and RC & computer control modes
    /// (happens at multiple levels between control loops)
    switch(armed_state){
      case ARMED:
        switch(composite_control_mode){
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

    // send serial sensor data
    // send low priority messages (e.g. param values)
    //  internal timers figure out what to send
  mavlink_stream(now);

  /// Post-Process
    // receive mavlink messages
  mavlink_receive();

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
