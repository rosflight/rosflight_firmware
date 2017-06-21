/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab, Provo UT
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "rosflight.h"

namespace rosflight
{

ROSflight::ROSflight(Board *_board, CommLink *_commlink)
{
  board_ = _board;
  commlink_ = _commlink;
}

// Initialization Routine
void ROSflight::rosflight_init(void)
{
  // Initialize the arming finite state machine
  fsm_.init(this);

  // Read EEPROM to get initial params
  params_.init(this);

  // Initialize Mixer
  mixer_.init(this);

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize PWM and RC
  rc_.init(this);

  // Initialize MAVlink Communication
  commlink_->init(board_, &params_, this);

  // Initialize Sensors
  sensors_.init(this);

  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Estimator
  estimator_.init(this);

  // Initialize Controller
  controller_.init(this);

  // Initialize the command muxer
  mux_.init(this);
}


// Main loop
void ROSflight::rosflight_run()
{
  /*********************/
  /***  Control Loop ***/
  /*********************/
  uint64_t start = board_->clock_micros();
  if (sensors_.update_sensors()) // 595 | 591 | 590 us
  {
    // If I have new IMU data, then perform control
    estimator_.run_estimator(); //  212 | 195 us (acc and gyro only, not exp propagation no quadratic integration)
    controller_.run_controller(); // 278 | 271
    mixer_.mix_output(); // 16 | 13 us
    loop_time_us = board_->clock_micros() - start;
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
//  // internal timers figure out what and when to send
  commlink_->stream(); // 165 | 27 | 2

  // receive mavlink messages
  commlink_->receive(); // 159 | 1 | 1

  // update the state machine, an internal timer runs this at a fixed rate
  fsm_.update_state(); // 108 | 1 | 1

  // get RC, an internal timer runs this every 20 ms (50 Hz)
  rc_.receive_rc(); // 42 | 2 | 1

  // update commands (internal logic tells whether or not we should do anything or not)
  mux_.mux_inputs(); // 6 | 1 | 1
}

uint32_t ROSflight::get_loop_time_us()
{
  return loop_time_us;
}

}
