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

#ifndef ROSFLIGHT_FIRMWARE_STATE_MANAGER_H
#define ROSFLIGHT_FIRMWARE_STATE_MANAGER_H

#include <stdint.h>

namespace rosflight_firmware
{

class ROSflight;

class StateManager
{

public:
  struct State
  {
    bool armed;
    bool failsafe;
    bool error;
    uint16_t error_codes;
  };

  enum Event
  {
    EVENT_INITIALIZED,
    EVENT_REQUEST_ARM,
    EVENT_REQUEST_DISARM,
    EVENT_RC_LOST,
    EVENT_RC_FOUND,
    EVENT_ERROR,
    EVENT_NO_ERROR,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_CALIBRATION_FAILED,
  };

  enum
  {
    ERROR_NONE = 0x0000,
    ERROR_INVALID_MIXER = 0x0001,
    ERROR_IMU_NOT_RESPONDING = 0x0002,
    ERROR_RC_LOST = 0x0004,
    ERROR_UNHEALTHY_ESTIMATOR = 0x0008,
    ERROR_TIME_GOING_BACKWARDS = 0x0010,
    ERROR_UNCALIBRATED_IMU = 0x0020,
  };

  StateManager(ROSflight& parent);
  void init();
  void run();

  inline const State& state() const { return state_; }

  void set_event(Event event);
  void set_error(uint16_t error);
  void clear_error(uint16_t error);

private:
  ROSflight& RF_;
  State state_;

  uint32_t next_led_blink_ms_ = 0;

  enum FsmState
  {
    FSM_STATE_INIT,
    FSM_STATE_PREFLIGHT,
    FSM_STATE_ARMED,
    FSM_STATE_ERROR,
    FSM_STATE_FAILSAFE,
    FSM_STATE_CALIBRATING
  };

  FsmState fsm_state_;
  void process_errors();

  void update_leds();
};

} // namespace rosflight_firmware

#endif // ROSFLIGHT_FIRMWARE_STATE_MANAGER_H
