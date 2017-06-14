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
#ifndef RC_H_
#define RC_H_

#include <stdint.h>
#include <stdbool.h>

#include "param.h"

#include "mux.h"

typedef enum
{
  RC_STICK_X,
  RC_STICK_Y,
  RC_STICK_Z,
  RC_STICK_F,
  RC_STICKS_COUNT
} rc_stick_t;

typedef enum
{
  RC_SWITCH_ARM,
  RC_SWITCH_ATT_OVERRIDE,
  RC_SWITCH_THROTTLE_OVERRIDE,
  RC_SWITCH_ATT_TYPE,
  RC_SWITCHES_COUNT
} rc_switch_t;

typedef enum
{
  PARALLEL_PWM,
  CPPM,
} rc_type_t;

/**
 * @brief Initialize the RC sticks and switches.
 *
 * Assign channels and other values to the RC sticks and switches based on input parameters.
 */
void init_rc(void);

/**
 * @brief Get current stick value for the given channel.
 * @param  channel The stick channel whose value you wish to retrieve.
 * @return         Normalized float of the current stick value of the channel.
 */
float rc_stick(rc_stick_t channel);

/**
 * @brief Get the current switch value for the given channel.
 * @param  channel The switch channel whose value you wish to retrieve.
 * @return         True if the switch is mapped and in its on state, otherwise false.
 */
bool rc_switch(rc_switch_t channel);

/**
 * @brief Check if the given switch is mapped to an RC channel
 * @param  channel The switch type to check.
 * @return         True if this switch type was mapped to a valid RC channel, otherwise false.
 */
bool rc_switch_mapped(rc_switch_t channel);

/**
 * @brief Receive new RC data and update local data members.
 *
 *  Maps channeled inputs from the RC controller to their proper data member values within this
 *  class every 20ms. Upon update, signals to the mux that a new command is waiting.
 *
 * @return False if it hasn't been 20ms since the last update, otherwise true.
 */
bool receive_rc();

#endif
