#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "param.h"
#include "sensors.h"
#include "state.h"
#include "estimator.h"

void setup(void)
{
  // Read EEPROM to get initial params
  init_params();

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize I2c
  i2cInit(I2CDEV_2);
  // Initialize PWM
  // Initialize Serial Communication
  init_mavlink();



  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  // Initialize Estimator
  state_t _current_state = init_estimator();
  // Initialize Controller
  // Initialize State Machine
}

void loop(void)
{
  uint32_t loop_time_us = micros();

  /// Pre-process
    // get looptime - store time in a global variable
    // dt = micros();
    // update sensors (only the ones that need updating)
    // sensors = update_sensors(dt);
  update_sensors(loop_time_us);

  /// Main Thread
    // Run Estimator - uses global sensor information
    // state = runEstimator(sensors, dt, state); <--- state has to be persistent

    /// Need to mix between RC and computer based on override mode and RC & computer control modes
    /// (happens at multiple levels between control loops)
    /// IF ARMED: <--armedState
      // IF ALT_MODE: <-- controlMode
        // Run Alt_Controller
        // thrust_c = runAltController(alt_c, state);
        // ATTITUDE_MODE = True
      // IF ATTITUDE_MODE:
        // Run Attitude Controller
        // omega_c = runAttController(theta_c, state);
        // RATE_MODE = True
      // IF RATE_MODE
        // Run Rate_Controller
        // tau_c = runRateController(omega_c, state);
      // Mix Output
      // motor_speeds = mixOutput(tau_c);
      // Send to Motors
      // write_motors_armed(motor_speeds);
  /// ELSE:
    // write_motors_disarmed(); <-- For spin when armed

    // send serial sensor data
    // send low priority messages (e.g. param values)
    //  internal timers figure out what to send
  mavlink_stream(loop_time_us);

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
