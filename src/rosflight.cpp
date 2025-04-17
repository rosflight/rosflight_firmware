/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
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

#include "param_listener.h"

namespace rosflight_firmware
{
ROSflight::ROSflight(Board & board, CommLinkInterface & comm_link)
    : board_(board)
    , comm_manager_(*this, comm_link)
    , params_(*this)
    , command_manager_(*this)
    , controller_(*this)
    , estimator_(*this)
    , mixer_(*this)
    , rc_(*this)
    , sensors_(*this)
    , state_manager_(*this)
{
  comm_link.set_listener(&comm_manager_);
  params_.set_listeners(param_listeners_, num_param_listeners_);
}

// Initialization Routine
void ROSflight::init()
{
  // Initialize the arming finite state machine
  state_manager_.init();

  // Read EEPROM to get initial params
  params_.init();

  // Initialize Mixer
  mixer_.init();

  /***********************/
  /***  Hardware Setup ***/
  /***********************/

  // Initialize PWM and RC
  rc_.init();

  // Initialize MAVlink Communication
  comm_manager_.init();

  // Initialize Sensors
  sensors_.init();

  /***********************/
  /***  Software Setup ***/
  /***********************/

  // Initialize Estimator
  estimator_.init();

  // Initialize Controller
  controller_.init();

  // Initialize the command muxer
  command_manager_.init();

  /***************************/
  /***  Hardfault Recovery ***/
  /***************************/

  state_manager_.check_backup_memory();
}

/**
 * @fn void run()
 * @brief Main Loop
 *
 */
void ROSflight::run()
{
  /*********************/
  /***  Control Loop ***/
  /*********************/
 
  got_flags got = sensors_.run(); // IMU, GNSS, Baro, Mag, Pitot, SONAR, Battery

  if (got.imu) {
    uint64_t start = board_.clock_micros();
    estimator_.run();
    controller_.run();
    mixer_.mix_output();
    board_.pwm_write_multi(mixer_.raw_outputs(), Mixer::NUM_TOTAL_OUTPUTS);
    loop_time_us = board_.clock_micros() - start;
  }

  /*********************/
  /***  Post-Process ***/
  /*********************/
  // internal timers figure out what and when to send
  comm_manager_.stream(got);

  // receive mavlink messages
  comm_manager_.receive();

  // update the state machine, an internal timer runs this at a fixed rate
  state_manager_.run();

  // get RC, synchronous with rc data acquisition
  if (got.rc) { rc_.run(); }

  // update commands (internal logic tells whether or not we should do anything or not)
  command_manager_.run();
}

uint32_t ROSflight::get_loop_time_us() { return loop_time_us; }

} // namespace rosflight_firmware
