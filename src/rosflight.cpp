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

  // Initialize Mixer (because parameter callbacks need to use it.)
  mixer_.init(board_, &mux_, &params_, &fsm_);

  // Read EEPROM to get initial params
  params_.init_params(board_, commlink_, &mixer_);

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize PWM and RC
  mixer_.init_PWM();
  rc_.init_rc(&fsm_, board_, &params_, &mux_);

  // Initialize MAVlink Communication
  commlink_->init(board_, &params_, &sensors_);

  // Initialize Sensors
  sensors_.init_sensors(board_, &params_, &estimator_);

  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Motor Mixing
  mixer_.init_mixing();

  // Initialize Estimator
  estimator_.init_estimator(&params_, &sensors_);

  // Initialize Controller
  controller_.init_controller(&fsm_, board_, &estimator_, &params_);

  // Initialize the arming finite state machine
  fsm_.init_mode(board_, &sensors_, &params_, &rc_);
}


// Main loop
void ROSflight::rosflight_run()
{
  /*********************/
  /***  Control Loop ***/
  /*********************/
  if (sensors_.update_sensors()) // 595 | 591 | 590 us
  {
    // If I have new IMU data, then perform control
    estimator_.run_estimator(); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
    controller_.run_controller(); // 278 | 271
    mixer_.mix_output(); // 16 | 13 us
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
//  // internal timers figure out what and when to send
  commlink_->stream(); // 165 | 27 | 2

  // receive mavlink messages
  commlink_->receive(); // 159 | 1 | 1

  // update the state machine, an internal timer runs this at a fixed rate
  fsm_.check_mode(); // 108 | 1 | 1

  // get RC, an internal timer runs this every 20 ms (50 Hz)
  rc_.receive_rc(); // 42 | 2 | 1

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_.mux_inputs(); // 6 | 1 | 1
}

}
