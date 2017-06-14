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

#ifdef __cplusplus
extern "C" {
#endif


#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  RATE,         // Channel is is in rate mode (mrad/s)
  ANGLE,        // Channel command is in angle mode (mrad)
  THROTTLE,     // Channel is direcly controlling throttle max/1000
  PASSTHROUGH,  // Channel directly passes PWM input to the mixer
} control_type_t;

typedef struct
{
  bool active;          // Whether or not the channel is active
  control_type_t type;  // What type the channel is
  float value;          // The value of the channel
} control_channel_t;

typedef struct
{
  control_channel_t x;
  control_channel_t y;
  control_channel_t z;
  control_channel_t F;
} control_t;

extern control_t _rc_control;
extern control_t _offboard_control;
extern control_t _combined_control;
extern control_t _failsafe_control;

extern bool _new_command;


/**
 * @brief Selects a combination of 3 possible control inputs for a single combined control output.
 *
 *  Selects between failsafe, offboard, and rc input control, and based off current state, muxes them into
 *  the #_combined_control extern.
 *
 * @return False if no new commands were triggered by the 3 control inputs, otherwise true.
 */
bool mux_inputs();

/**
 * @brief Check if the RC is currently overriding all other commands.
 * @return True if the RC is currently overriding other commands, otherwise false.
 */
bool rc_override_active();

/**
 * @brief Checks if any of the channels are currently controlled by the offboard controller.
 * @return True if one of the muxed offboard channels are active, otherwise false.
 */
bool offboard_control_active();

#ifdef __cplusplus
}
#endif

