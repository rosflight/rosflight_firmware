#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "param.h"
#include "sensors.h"

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



  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  // Initialize Estimator
  // Initialize Controller
  // Initialize State Machine
}

void loop(void)
{
  /// Pre-process
    // get looptime - store time in a global variable
    // dt = micros();
    // update sensors (only the ones that need updating)
    // sensors = update_sensors(dt);
    update_sensors(micros());

  /// Main Thread
    // Run Estimator - uses global sensor information
    // state = runEstimator(sensors, dt, state); <--- state has to be persistent

    /// IF ARMED:
      // IF ALT_MODE:
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


  /// Post-Process
    // update RC data
    // RC = updateRC();
    // commands from the computer will be updated by callbacks

    // if it has been at least 50 Hz
    // update RC params <-- params that can be set by the safety pilot
      // This will write ARMED, ALT_MODE, ATTITUDE_MODE, RATE_MODE
      // updateRCParams();
    // update computer params
    // updateParams();

    // send serial senor data
    // sendSensorData();

    // Figure Out Commands for next loop
    // IF ARMED
      // IF ALT_MODE:
        // alt_c = getCommands();  <-- pulls from the global, and chooses the appropriate command
      // IF ATTITUDE_MODE:
        // theta_c = getCommands();
      // IF RATE_MODE:
        // omega_c = getCommands();
}
