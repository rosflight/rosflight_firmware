#include "rosflight.h"

namespace rosflight {

ROSflight::ROSflight(Board *_board, CommLink *_commlink)
{
  board_ = _board;
  commlink_ = _commlink;
}

// Initialization Routine
void ROSflight::rosflight_init(void)
{
  board_->init_board();

  // Read EEPROM to get initial params
  params_.init_params(board_, commlink_);

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

//  // Initialize PWM and RC
//  init_PWM();
//  init_rc();

  // Initialize MAVlink Communication
  commlink_->init(board_, &params_, &sensors_);

//  // Initialize Sensors
//  sensors_.init_sensors(board_, &params_);

//  /***********************/
//  /***  Software Setup ***/
//  /***********************/

//  // Initialize Motor Mixing
//  init_mixing();

//  // Initizlie Controller
//  init_controller();

//  // Initialize Estimator
//  // mat_exp <- greater accuracy, but adds ~90 us
//  // quadratic_integration <- some additional accuracy, adds ~20 us
//  // accelerometer correction <- if using angle mode, this is required, adds ~70 us
  estimator_.init_estimator(&params_, &sensors_);
  fsm_.init_mode(board_, &sensors_, &params_);
}


// Main loop
void ROSflight::rosflight_run()
{
//  /*********************/
//  /***  Control Loop ***/
//  /*********************/
  if (sensors_.update_sensors()) // 595 | 591 | 590 us
  {
    // If I have new IMU data, then perform control
    estimator_.run_estimator(); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
//    run_controller(); // 278 | 271
//    mix_output(); // 16 | 13 us
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
//  // internal timers figure out what and when to send
  commlink_->stream(); // 165 | 27 | 2

//  // receive mavlink messages
//  mavlink_receive(); // 159 | 1 | 1

//  // update the armed_states, an internal timer runs this at a fixed rate
  fsm_.check_mode(); // 108 | 1 | 1

//  // get RC, an internal timer runs this every 20 ms (50 Hz)
//  receive_rc(); // 42 | 2 | 1

//  // update commands (internal logic tells whether or not we should do anything or not)
//  mux_inputs(); // 6 | 1 | 1
}

}
