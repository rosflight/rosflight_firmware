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

#include "param.h"

#include "board.h"
#include "mixer.h"

#include "rosflight.h"

#include <cstdint>
#include <cstring>

namespace rosflight_firmware
{
Params::Params(ROSflight & _rf)
    : RF_(_rf)
    , listeners_(nullptr)
    , num_listeners_(0)
{}

// local function definitions
void Params::init_param_int(uint16_t id, const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  // copy cstr including '\0' or until maxlen
  const uint8_t len = (strlen(name) >= PARAMS_NAME_LENGTH) ? PARAMS_NAME_LENGTH : strlen(name) + 1;
  memcpy(params.names[id], name, len);
  params.values[id].ivalue = value;
  params.types[id] = PARAM_TYPE_INT32;
}

void Params::init_param_float(uint16_t id, const char name[PARAMS_NAME_LENGTH], float value)
{
  // copy cstr including '\0' or until maxlen
  const uint8_t len = (strlen(name) >= PARAMS_NAME_LENGTH) ? PARAMS_NAME_LENGTH : strlen(name) + 1;
  memcpy(params.names[id], name, len);
  params.values[id].fvalue = value;
  params.types[id] = PARAM_TYPE_FLOAT;
}

uint8_t Params::compute_checksum(void)
{
  uint8_t chk = 0;
  const char * p;

  for (p = reinterpret_cast<const char *>(&params.values);
       p < reinterpret_cast<const char *>(&params.values) + 4 * PARAMS_COUNT; p++) {
    chk ^= *p;
  }
  for (p = reinterpret_cast<const char *>(&params.names);
       p < reinterpret_cast<const char *>(&params.names) + PARAMS_COUNT * PARAMS_NAME_LENGTH; p++) {
    chk ^= *p;
  }
  for (p = reinterpret_cast<const char *>(&params.types);
       p < reinterpret_cast<const char *>(&params.types) + PARAMS_COUNT; p++) {
    chk ^= *p;
  }

  return chk;
}

// function definitions
void Params::init()
{
  RF_.board_.memory_init();
  if (!read()) {
    RF_.comm_manager_.log(CommLinkInterface::LogSeverity::LOG_WARNING,
                          "Unable to load parameters; using default values");
    set_defaults();
    write();
  }
}

// clang-format off
void Params::set_defaults(void)
{
  /******************************/
  /*** HARDWARE CONFIGURATION ***/
  /******************************/
  init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600); // Baud rate of MAVlink communication with companion computer | 9600 | 921600
  init_param_int(PARAM_SERIAL_DEVICE, "SERIAL_DEVICE", 0); // Serial Port (for supported devices) | 0 | 3

  init_param_int(PARAM_NUM_MOTORS, "NUM_MOTORS", 4); // Number of vertical-facing motors on the vehicle | 1 | 8
  init_param_float(PARAM_MOTOR_RESISTANCE, "MOTOR_RESISTANCE", 0.042f); // Electrical resistance of the motor windings (ohms) | 0 | 1000.0
  init_param_float(PARAM_MOTOR_KV, "MOTOR_KV", 0.01706f); // Back emf constant of the motor in SI units (V/rad/s) | 0 | 1000.0
  init_param_float(PARAM_MOTOR_NO_LOAD_CURRENT, "NO_LOAD_CURRENT", 1.5); // No-load current of the motor in amps | 0 | 1000.0
  init_param_float(PARAM_PROP_DIAMETER, "PROP_DIAMETER", 0.381f); // Diameter of the propeller in meters | 0 | 1.0
  init_param_float(PARAM_PROP_CT, "PROP_CT", 0.075f); // Thrust coefficient of the propeller | 0 | 100.0
  init_param_float(PARAM_PROP_CQ, "PROP_CQ", 0.0045f); // Torque coefficient of the propeller | 0 | 100.0
  init_param_float(PARAM_BATT_VOLT_MAX, "BATT_VOLT_MAX", 25.0f); // Maximum voltage of the battery (V) | 0 | 100.0
  init_param_int(PARAM_USE_MOTOR_PARAMETERS, "USE_MOTOR_PARAM", false);   // Flag to use motor parameters in the mixer | 0 | 1

  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_0, "PRI_MIXER_OUT_0", 0); // Output type of mixer output 0. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_1, "PRI_MIXER_OUT_1", 0); // Output type of mixer output 1. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_2, "PRI_MIXER_OUT_2", 0); // Output type of mixer output 2. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_3, "PRI_MIXER_OUT_3", 0); // Output type of mixer output 3. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_4, "PRI_MIXER_OUT_4", 0); // Output type of mixer output 4. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_5, "PRI_MIXER_OUT_5", 0); // Output type of mixer output 5. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_6, "PRI_MIXER_OUT_6", 0); // Output type of mixer output 6. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_7, "PRI_MIXER_OUT_7", 0); // Output type of mixer output 7. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_8, "PRI_MIXER_OUT_8", 0); // Output type of mixer output 8. | 0 | 3
  init_param_int(PARAM_PRIMARY_MIXER_OUTPUT_9, "PRI_MIXER_OUT_9", 0); // Output type of mixer output 9. | 0 | 3

  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_0, "PRI_MIXER_PWM_0", 0); // PWM frequenct output for mixer output 0 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_1, "PRI_MIXER_PWM_1", 0); // PWM frequenct output for mixer output 1 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_2, "PRI_MIXER_PWM_2", 0); // PWM frequenct output for mixer output 2 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_3, "PRI_MIXER_PWM_3", 0); // PWM frequenct output for mixer output 3 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_4, "PRI_MIXER_PWM_4", 0); // PWM frequenct output for mixer output 4 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_5, "PRI_MIXER_PWM_5", 0); // PWM frequenct output for mixer output 5 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_6, "PRI_MIXER_PWM_6", 0); // PWM frequenct output for mixer output 6 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_7, "PRI_MIXER_PWM_7", 0); // PWM frequenct output for mixer output 7 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_8, "PRI_MIXER_PWM_8", 0); // PWM frequenct output for mixer output 8 | 0 | 490
  init_param_float(PARAM_PRIMARY_MIXER_PWM_RATE_9, "PRI_MIXER_PWM_9", 0); // PWM frequenct output for mixer output 9 | 0 | 490

  init_param_float(PARAM_PRIMARY_MIXER_0_0, "PRI_MIXER_0_0", 0.0f); // Value of the custom mixer at element [0,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_0, "PRI_MIXER_1_0", 0.0f); // Value of the custom mixer at element [1,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_0, "PRI_MIXER_2_0", 0.0f); // Value of the custom mixer at element [2,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_0, "PRI_MIXER_3_0", 0.0f); // Value of the custom mixer at element [3,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_0, "PRI_MIXER_4_0", 0.0f); // Value of the custom mixer at element [4,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_0, "PRI_MIXER_5_0", 0.0f); // Value of the custom mixer at element [5,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_0, "PRI_MIXER_6_0", 0.0f); // Value of the custom mixer at element [6,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_0, "PRI_MIXER_7_0", 0.0f); // Value of the custom mixer at element [7,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_0, "PRI_MIXER_8_0", 0.0f); // Value of the custom mixer at element [8,0] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_0, "PRI_MIXER_9_0", 0.0f); // Value of the custom mixer at element [9,0] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_1, "PRI_MIXER_0_1", 0.0f); // Value of the custom mixer at element [0,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_1, "PRI_MIXER_1_1", 0.0f); // Value of the custom mixer at element [1,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_1, "PRI_MIXER_2_1", 0.0f); // Value of the custom mixer at element [2,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_1, "PRI_MIXER_3_1", 0.0f); // Value of the custom mixer at element [3,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_1, "PRI_MIXER_4_1", 0.0f); // Value of the custom mixer at element [4,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_1, "PRI_MIXER_5_1", 0.0f); // Value of the custom mixer at element [5,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_1, "PRI_MIXER_6_1", 0.0f); // Value of the custom mixer at element [6,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_1, "PRI_MIXER_7_1", 0.0f); // Value of the custom mixer at element [7,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_1, "PRI_MIXER_8_1", 0.0f); // Value of the custom mixer at element [8,1] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_1, "PRI_MIXER_9_1", 0.0f); // Value of the custom mixer at element [9,1] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_2, "PRI_MIXER_0_2", 0.0f); // Value of the custom mixer at element [0,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_2, "PRI_MIXER_1_2", 0.0f); // Value of the custom mixer at element [1,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_2, "PRI_MIXER_2_2", 0.0f); // Value of the custom mixer at element [2,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_2, "PRI_MIXER_3_2", 0.0f); // Value of the custom mixer at element [3,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_2, "PRI_MIXER_4_2", 0.0f); // Value of the custom mixer at element [4,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_2, "PRI_MIXER_5_2", 0.0f); // Value of the custom mixer at element [5,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_2, "PRI_MIXER_6_2", 0.0f); // Value of the custom mixer at element [6,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_2, "PRI_MIXER_7_2", 0.0f); // Value of the custom mixer at element [7,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_2, "PRI_MIXER_8_2", 0.0f); // Value of the custom mixer at element [8,2] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_2, "PRI_MIXER_9_2", 0.0f); // Value of the custom mixer at element [9,2] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_3, "PRI_MIXER_0_3", 0.0f); // Value of the custom mixer at element [0,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_3, "PRI_MIXER_1_3", 0.0f); // Value of the custom mixer at element [1,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_3, "PRI_MIXER_2_3", 0.0f); // Value of the custom mixer at element [2,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_3, "PRI_MIXER_3_3", 0.0f); // Value of the custom mixer at element [3,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_3, "PRI_MIXER_4_3", 0.0f); // Value of the custom mixer at element [4,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_3, "PRI_MIXER_5_3", 0.0f); // Value of the custom mixer at element [5,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_3, "PRI_MIXER_6_3", 0.0f); // Value of the custom mixer at element [6,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_3, "PRI_MIXER_7_3", 0.0f); // Value of the custom mixer at element [7,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_3, "PRI_MIXER_8_3", 0.0f); // Value of the custom mixer at element [8,3] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_3, "PRI_MIXER_9_3", 0.0f); // Value of the custom mixer at element [9,3] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_4, "PRI_MIXER_0_4", 0.0f); // Value of the custom mixer at element [0,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_4, "PRI_MIXER_1_4", 0.0f); // Value of the custom mixer at element [1,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_4, "PRI_MIXER_2_4", 0.0f); // Value of the custom mixer at element [2,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_4, "PRI_MIXER_3_4", 0.0f); // Value of the custom mixer at element [3,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_4, "PRI_MIXER_4_4", 0.0f); // Value of the custom mixer at element [4,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_4, "PRI_MIXER_5_4", 0.0f); // Value of the custom mixer at element [5,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_4, "PRI_MIXER_6_4", 0.0f); // Value of the custom mixer at element [6,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_4, "PRI_MIXER_7_4", 0.0f); // Value of the custom mixer at element [7,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_4, "PRI_MIXER_8_4", 0.0f); // Value of the custom mixer at element [8,4] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_4, "PRI_MIXER_9_4", 0.0f); // Value of the custom mixer at element [9,4] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_5, "PRI_MIXER_0_5", 0.0f); // Value of the custom mixer at element [0,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_5, "PRI_MIXER_1_5", 0.0f); // Value of the custom mixer at element [1,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_5, "PRI_MIXER_2_5", 0.0f); // Value of the custom mixer at element [2,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_5, "PRI_MIXER_3_5", 0.0f); // Value of the custom mixer at element [3,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_5, "PRI_MIXER_4_5", 0.0f); // Value of the custom mixer at element [4,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_5, "PRI_MIXER_5_5", 0.0f); // Value of the custom mixer at element [5,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_5, "PRI_MIXER_6_5", 0.0f); // Value of the custom mixer at element [6,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_5, "PRI_MIXER_7_5", 0.0f); // Value of the custom mixer at element [7,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_5, "PRI_MIXER_8_5", 0.0f); // Value of the custom mixer at element [8,5] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_5, "PRI_MIXER_9_5", 0.0f); // Value of the custom mixer at element [9,5] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_6, "PRI_MIXER_0_6", 0.0f); // Value of the custom mixer at element [0,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_6, "PRI_MIXER_1_6", 0.0f); // Value of the custom mixer at element [1,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_6, "PRI_MIXER_2_6", 0.0f); // Value of the custom mixer at element [2,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_6, "PRI_MIXER_3_6", 0.0f); // Value of the custom mixer at element [3,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_6, "PRI_MIXER_4_6", 0.0f); // Value of the custom mixer at element [4,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_6, "PRI_MIXER_5_6", 0.0f); // Value of the custom mixer at element [5,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_6, "PRI_MIXER_6_6", 0.0f); // Value of the custom mixer at element [6,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_6, "PRI_MIXER_7_6", 0.0f); // Value of the custom mixer at element [7,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_6, "PRI_MIXER_8_6", 0.0f); // Value of the custom mixer at element [8,6] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_6, "PRI_MIXER_9_6", 0.0f); // Value of the custom mixer at element [9,6] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_7, "PRI_MIXER_0_7", 0.0f); // Value of the custom mixer at element [0,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_7, "PRI_MIXER_1_7", 0.0f); // Value of the custom mixer at element [1,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_7, "PRI_MIXER_2_7", 0.0f); // Value of the custom mixer at element [2,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_7, "PRI_MIXER_3_7", 0.0f); // Value of the custom mixer at element [3,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_7, "PRI_MIXER_4_7", 0.0f); // Value of the custom mixer at element [4,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_7, "PRI_MIXER_5_7", 0.0f); // Value of the custom mixer at element [5,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_7, "PRI_MIXER_6_7", 0.0f); // Value of the custom mixer at element [6,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_7, "PRI_MIXER_7_7", 0.0f); // Value of the custom mixer at element [7,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_7, "PRI_MIXER_8_7", 0.0f); // Value of the custom mixer at element [8,7] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_7, "PRI_MIXER_9_7", 0.0f); // Value of the custom mixer at element [9,7] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_8, "PRI_MIXER_0_8", 0.0f); // Value of the custom mixer at element [0,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_8, "PRI_MIXER_1_8", 0.0f); // Value of the custom mixer at element [1,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_8, "PRI_MIXER_2_8", 0.0f); // Value of the custom mixer at element [2,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_8, "PRI_MIXER_3_8", 0.0f); // Value of the custom mixer at element [3,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_8, "PRI_MIXER_4_8", 0.0f); // Value of the custom mixer at element [4,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_8, "PRI_MIXER_5_8", 0.0f); // Value of the custom mixer at element [5,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_8, "PRI_MIXER_6_8", 0.0f); // Value of the custom mixer at element [6,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_8, "PRI_MIXER_7_8", 0.0f); // Value of the custom mixer at element [7,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_8, "PRI_MIXER_8_8", 0.0f); // Value of the custom mixer at element [8,8] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_8, "PRI_MIXER_9_8", 0.0f); // Value of the custom mixer at element [9,8] | -inf | inf

  init_param_float(PARAM_PRIMARY_MIXER_0_9, "PRI_MIXER_0_9", 0.0f); // Value of the custom mixer at element [0,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_1_9, "PRI_MIXER_1_9", 0.0f); // Value of the custom mixer at element [1,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_2_9, "PRI_MIXER_2_9", 0.0f); // Value of the custom mixer at element [2,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_3_9, "PRI_MIXER_3_9", 0.0f); // Value of the custom mixer at element [3,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_4_9, "PRI_MIXER_4_9", 0.0f); // Value of the custom mixer at element [4,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_5_9, "PRI_MIXER_5_9", 0.0f); // Value of the custom mixer at element [5,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_6_9, "PRI_MIXER_6_9", 0.0f); // Value of the custom mixer at element [6,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_7_9, "PRI_MIXER_7_9", 0.0f); // Value of the custom mixer at element [7,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_8_9, "PRI_MIXER_8_9", 0.0f); // Value of the custom mixer at element [8,9] | -inf | inf
  init_param_float(PARAM_PRIMARY_MIXER_9_9, "PRI_MIXER_9_9", 0.0f); // Value of the custom mixer at element [9,9] | -inf | inf

  init_param_float(PARAM_SECONDARY_MIXER_0_0, "SEC_MIXER_0_0", 0.0f); // Value of the custom mixer at element [0,0] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_0, "SEC_MIXER_1_0", 0.0f); // Value of the custom mixer at element [1,0] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_0, "SEC_MIXER_2_0", 0.0f); // Value of the custom mixer at element [2,0] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_0, "SEC_MIXER_3_0", 0.0f); // Value of the custom mixer at element [3,0] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_0, "SEC_MIXER_4_0", 0.0f); // Value of the custom mixer at element [4,0] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_0, "SEC_MIXER_5_0", 0.0f); // Value of the custom mixer at element [5,0] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_6_0, "SEC_MIXER_6_0", 0.0f); // Value of the custom mixer at element [6,0] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_0, "SEC_MIXER_7_0", 0.0f); // Value of the custom mixer at element [7,0] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_8_0, "SEC_MIXER_8_0", 0.0f); // Value of the custom mixer at element [8,0] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_9_0, "SEC_MIXER_9_0", 0.0f); // Value of the custom mixer at element [9,0] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_1, "SEC_MIXER_0_1", 0.0f); // Value of the custom mixer at element [0,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_1, "SEC_MIXER_1_1", 0.0f); // Value of the custom mixer at element [1,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_1, "SEC_MIXER_2_1", 0.0f); // Value of the custom mixer at element [2,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_1, "SEC_MIXER_3_1", 0.0f); // Value of the custom mixer at element [3,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_1, "SEC_MIXER_4_1", 0.0f); // Value of the custom mixer at element [4,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_1, "SEC_MIXER_5_1", 0.0f); // Value of the custom mixer at element [5,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_1, "SEC_MIXER_6_1", 0.0f); // Value of the custom mixer at element [6,1] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_1, "SEC_MIXER_7_1", 0.0f); // Value of the custom mixer at element [7,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_8_1, "SEC_MIXER_8_1", 0.0f); // Value of the custom mixer at element [8,1] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_9_1, "SEC_MIXER_9_1", 0.0f); // Value of the custom mixer at element [9,1] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_2, "SEC_MIXER_0_2", 0.0f); // Value of the custom mixer at element [0,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_2, "SEC_MIXER_1_2", 0.0f); // Value of the custom mixer at element [1,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_2, "SEC_MIXER_2_2", 0.0f); // Value of the custom mixer at element [2,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_2, "SEC_MIXER_3_2", 0.0f); // Value of the custom mixer at element [3,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_2, "SEC_MIXER_4_2", 0.0f); // Value of the custom mixer at element [4,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_2, "SEC_MIXER_5_2", 0.0f); // Value of the custom mixer at element [5,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_2, "SEC_MIXER_6_2", 0.0f); // Value of the custom mixer at element [6,2] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_2, "SEC_MIXER_7_2", 0.0f); // Value of the custom mixer at element [7,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_8_2, "SEC_MIXER_8_2", 0.0f); // Value of the custom mixer at element [8,2] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_9_2, "SEC_MIXER_9_2", 0.0f); // Value of the custom mixer at element [9,2] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_3, "SEC_MIXER_0_3", 0.0f); // Value of the custom mixer at element [0,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_3, "SEC_MIXER_1_3", 0.0f); // Value of the custom mixer at element [1,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_3, "SEC_MIXER_2_3", 0.0f); // Value of the custom mixer at element [2,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_3, "SEC_MIXER_3_3", 0.0f); // Value of the custom mixer at element [3,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_3, "SEC_MIXER_4_3", 0.0f); // Value of the custom mixer at element [4,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_3, "SEC_MIXER_5_3", 0.0f); // Value of the custom mixer at element [5,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_3, "SEC_MIXER_6_3", 0.0f); // Value of the custom mixer at element [6,3] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_3, "SEC_MIXER_7_3", 0.0f); // Value of the custom mixer at element [7,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_8_3, "SEC_MIXER_8_3", 0.0f); // Value of the custom mixer at element [8,3] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_9_3, "SEC_MIXER_9_3", 0.0f); // Value of the custom mixer at element [9,3] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_4, "SEC_MIXER_0_4", 0.0f); // Value of the custom mixer at element [0,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_4, "SEC_MIXER_1_4", 0.0f); // Value of the custom mixer at element [1,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_4, "SEC_MIXER_2_4", 0.0f); // Value of the custom mixer at element [2,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_4, "SEC_MIXER_3_4", 0.0f); // Value of the custom mixer at element [3,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_4, "SEC_MIXER_4_4", 0.0f); // Value of the custom mixer at element [4,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_4, "SEC_MIXER_5_4", 0.0f); // Value of the custom mixer at element [5,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_4, "SEC_MIXER_6_4", 0.0f); // Value of the custom mixer at element [6,4] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_4, "SEC_MIXER_7_4", 0.0f); // Value of the custom mixer at element [7,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_8_4, "SEC_MIXER_8_4", 0.0f); // Value of the custom mixer at element [8,4] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_9_4, "SEC_MIXER_9_4", 0.0f); // Value of the custom mixer at element [9,4] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_5, "SEC_MIXER_0_5", 0.0f); // Value of the custom mixer at element [0,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_5, "SEC_MIXER_1_5", 0.0f); // Value of the custom mixer at element [1,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_5, "SEC_MIXER_2_5", 0.0f); // Value of the custom mixer at element [2,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_5, "SEC_MIXER_3_5", 0.0f); // Value of the custom mixer at element [3,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_5, "SEC_MIXER_4_5", 0.0f); // Value of the custom mixer at element [4,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_5, "SEC_MIXER_5_5", 0.0f); // Value of the custom mixer at element [5,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_5, "SEC_MIXER_6_5", 0.0f); // Value of the custom mixer at element [6,5] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_5, "SEC_MIXER_7_5", 0.0f); // Value of the custom mixer at element [7,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_8_5, "SEC_MIXER_8_5", 0.0f); // Value of the custom mixer at element [8,5] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_9_5, "SEC_MIXER_9_5", 0.0f); // Value of the custom mixer at element [9,5] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_6, "SEC_MIXER_0_6", 0.0f); // Value of the custom mixer at element [0,6] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_6, "SEC_MIXER_1_6", 0.0f); // Value of the custom mixer at element [1,6] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_6, "SEC_MIXER_2_6", 0.0f); // Value of the custom mixer at element [2,6] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_6, "SEC_MIXER_3_6", 0.0f); // Value of the custom mixer at element [3,6] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_6, "SEC_MIXER_4_6", 0.0f); // Value of the custom mixer at element [4,6] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_6, "SEC_MIXER_5_6", 0.0f); // Value of the custom mixer at element [5,6] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_6, "SEC_MIXER_6_6", 0.0f); // Value of the custom mixer at element [6,6] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_6, "SEC_MIXER_7_6", 0.0f); // Value of the custom mixer at element [7,6] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_8_6, "SEC_MIXER_8_6", 0.0f); // Value of the custom mixer at element [8,6] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_9_6, "SEC_MIXER_9_6", 0.0f); // Value of the custom mixer at element [9,6] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_7, "SEC_MIXER_0_7", 0.0f); // Value of the custom mixer at element [0,7] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_7, "SEC_MIXER_1_7", 0.0f); // Value of the custom mixer at element [1,7] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_7, "SEC_MIXER_2_7", 0.0f); // Value of the custom mixer at element [2,7] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_7, "SEC_MIXER_3_7", 0.0f); // Value of the custom mixer at element [3,7] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_7, "SEC_MIXER_4_7", 0.0f); // Value of the custom mixer at element [4,7] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_7, "SEC_MIXER_5_7", 0.0f); // Value of the custom mixer at element [5,7] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_7, "SEC_MIXER_6_7", 0.0f); // Value of the custom mixer at element [6,7] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_7, "SEC_MIXER_7_7", 0.0f); // Value of the custom mixer at element [7,7] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_8_7, "SEC_MIXER_8_7", 0.0f); // Value of the custom mixer at element [8,7] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_9_7, "SEC_MIXER_9_7", 0.0f); // Value of the custom mixer at element [9,7] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_8, "SEC_MIXER_0_8", 0.0f); // Value of the custom mixer at element [0,8] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_8, "SEC_MIXER_1_8", 0.0f); // Value of the custom mixer at element [1,8] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_8, "SEC_MIXER_2_8", 0.0f); // Value of the custom mixer at element [2,8] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_8, "SEC_MIXER_3_8", 0.0f); // Value of the custom mixer at element [3,8] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_8, "SEC_MIXER_4_8", 0.0f); // Value of the custom mixer at element [4,8] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_8, "SEC_MIXER_5_8", 0.0f); // Value of the custom mixer at element [5,8] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_8, "SEC_MIXER_6_8", 0.0f); // Value of the custom mixer at element [6,8] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_8, "SEC_MIXER_7_8", 0.0f); // Value of the custom mixer at element [7,8] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_8_8, "SEC_MIXER_8_8", 0.0f); // Value of the custom mixer at element [8,8] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_9_8, "SEC_MIXER_9_8", 0.0f); // Value of the custom mixer at element [9,8] | -inf | inf
                                                                                                                                 
  init_param_float(PARAM_SECONDARY_MIXER_0_9, "SEC_MIXER_0_9", 0.0f); // Value of the custom mixer at element [0,9] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_1_9, "SEC_MIXER_1_9", 0.0f); // Value of the custom mixer at element [1,9] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_2_9, "SEC_MIXER_2_9", 0.0f); // Value of the custom mixer at element [2,9] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_3_9, "SEC_MIXER_3_9", 0.0f); // Value of the custom mixer at element [3,9] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_4_9, "SEC_MIXER_4_9", 0.0f); // Value of the custom mixer at element [4,9] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_5_9, "SEC_MIXER_5_9", 0.0f); // Value of the custom mixer at element [5,9] | -inf | inf 
  init_param_float(PARAM_SECONDARY_MIXER_6_9, "SEC_MIXER_6_9", 0.0f); // Value of the custom mixer at element [6,9] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_7_9, "SEC_MIXER_7_9", 0.0f); // Value of the custom mixer at element [7,9] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_8_9, "SEC_MIXER_8_9", 0.0f); // Value of the custom mixer at element [8,9] | -inf | inf
  init_param_float(PARAM_SECONDARY_MIXER_9_9, "SEC_MIXER_9_9", 0.0f); // Value of the custom mixer at element [9,9] | -inf | inf

  /*****************************/
  /*** MAVLINK CONFIGURATION ***/
  /*****************************/
  init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1); // Mavlink System ID  | 1 | 255

  /********************************/
  /*** CONTROLLER CONFIGURATION ***/
  /********************************/
  init_param_float(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", 0.070f); // Roll Rate Proportional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", 0.000f); // Roll Rate Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_RATE_D, "PID_ROLL_RATE_D", 0.000f); // Roll Rate Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", 0.070f);  // Pitch Rate Proportional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", 0.0000f); // Pitch Rate Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_RATE_D, "PID_PITCH_RATE_D", 0.0000f); // Pitch Rate Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", 0.25f);   // Yaw Rate Proportional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", 0.0f);  // Yaw Rate Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_YAW_RATE_D, "PID_YAW_RATE_D", 0.0f);  // Yaw Rate Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", 0.15f);   // Roll Angle Proportional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", 0.0f);   // Roll Angle Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", 0.05f);  // Roll Angle Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", 0.15f);  // Pitch Angle Proportional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", 0.0f);  // Pitch Angle Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", 0.05f); // Pitch Angle Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_X_EQ_TORQUE, "X_EQ_TORQUE", 0.0f); // Equilibrium torque added to output of controller on x axis | -1.0 | 1.0
  init_param_float(PARAM_Y_EQ_TORQUE, "Y_EQ_TORQUE", 0.0f); // Equilibrium torque added to output of controller on y axis | -1.0 | 1.0
  init_param_float(PARAM_Z_EQ_TORQUE, "Z_EQ_TORQUE", 0.0f); // Equilibrium torque added to output of controller on z axis | -1.0 | 1.0

  init_param_float(PARAM_PID_TAU, "PID_TAU", 0.05f); // Dirty Derivative time constant - See controller documentation | 0.0 | 1.0


  /*************************/
  /*** PWM CONFIGURATION ***/
  /*************************/
  init_param_float(PARAM_MOTOR_IDLE_THROTTLE, "MOTOR_IDLE_THR", 0.1); // min throttle command sent to motors when armed (Set above 0.1 to spin when armed) | 0.0 | 1.0
  init_param_float(PARAM_FAILSAFE_THROTTLE, "FAILSAFE_THR", -1.0); // Throttle sent to motors in failsafe condition (set just below hover throttle) | 0.0 | 1.0
  init_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", true); // Enforce MOTOR_IDLE_THR | 0 | 1

  /*******************************/
  /*** ESTIMATOR CONFIGURATION ***/
  /*******************************/
  init_param_int(PARAM_INIT_TIME, "FILT_INIT_T", 3000); // Time in ms to initialize estimator | 0 | 100000
  init_param_float(PARAM_FILTER_KP_ACC, "FILT_ACC_KP", 0.5f); // estimator proportional gain on accel-based error - See estimator documentation | 0 | 10.0
  init_param_float(PARAM_FILTER_KI, "FILT_KI", 0.01f); // estimator integral gain - See estimator documentation | 0 | 1.0
  init_param_float(PARAM_FILTER_KP_EXT, "FILT_EXT_KP", 1.5f); // estimator proportional gain on external attitude-based error - See estimator documentation | 0 | 10.0
  init_param_float(PARAM_FILTER_ACCEL_MARGIN, "FILT_ACC_MARGIN", 0.1f); // allowable accel norm margin around 1g to determine if accel is usable | 0 | 1.0

  init_param_int(PARAM_FILTER_USE_QUAD_INT, "FILT_QUAD_INT", 1); // Perform a quadratic averaging of LPF gyro data prior to integration (adds ~20 us to estimation loop on F1 processors) | 0 | 1
  init_param_int(PARAM_FILTER_USE_MAT_EXP, "FILT_MAT_EXP", 1); // 1 - Use matrix exponential to improve gyro integration (adds ~90 us to estimation loop in F1 processors) 0 - use euler integration | 0 | 1
  init_param_int(PARAM_FILTER_USE_ACC, "FILT_USE_ACC", 1);  // Use accelerometer to correct gyro integration drift (adds ~70 us to estimation loop) | 0 | 1

  init_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, "ARM_CAL_GYRO", false); // True if desired to calibrate gyros on arm | 0 | 1

  init_param_float(PARAM_GYRO_XY_ALPHA, "GYRO_XY_LPF", 0.3f); // Low-pass filter constant on gyro X and Y axes - See estimator documentation | 0 | 1.0
  init_param_float(PARAM_GYRO_Z_ALPHA, "GYRO_Z_LPF", 0.3f); // Low-pass filter constant on gyro Z axis - See estimator documentation | 0 | 1.0
  init_param_float(PARAM_ACC_ALPHA, "ACC_LPF", 0.5f); // Low-pass filter constant on all accel axes - See estimator documentation | 0 | 1.0

  init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0.0f); // Constant x-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0.0f); // Constant y-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0.0f); // Constant z-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0.0f); // Constant x-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0.0f); // Constant y-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0.0f); // Constant z-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_X_TEMP_COMP,  "ACC_X_TEMP_COMP", 0.0f); // Linear x-axis temperature compensation constant | -2.0 | 2.0
  init_param_float(PARAM_ACC_Y_TEMP_COMP,  "ACC_Y_TEMP_COMP", 0.0f); // Linear y-axis temperature compensation constant | -2.0 | 2.0
  init_param_float(PARAM_ACC_Z_TEMP_COMP,  "ACC_Z_TEMP_COMP", 0.0f); // Linear z-axis temperature compensation constant | -2.0 | 2.0

  init_param_float(PARAM_MAG_A00_COMP,  "MAG_CAL_A00", 1.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A01_COMP,  "MAG_CAL_A01", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A02_COMP,  "MAG_CAL_A02", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A10_COMP,  "MAG_CAL_A10", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A11_COMP,  "MAG_CAL_A11", 1.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A12_COMP,  "MAG_CAL_A12", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A20_COMP,  "MAG_CAL_A20", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A21_COMP,  "MAG_CAL_A21", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A22_COMP,  "MAG_CAL_A22", 1.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_X_BIAS,  "MAG_X_BIAS", 0.0f); // Hard iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_Y_BIAS,  "MAG_Y_BIAS", 0.0f); // Hard iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_Z_BIAS,  "MAG_Z_BIAS", 0.0f); // Hard iron compensation constant | -999.0 | 999.0

  init_param_float(PARAM_BARO_BIAS, "BARO_BIAS", 0.0f); // Barometer measurement bias (Pa) | 0 | inf
  init_param_float(PARAM_GROUND_LEVEL, "GROUND_LEVEL", 1387.0f); // Altitude of ground level (m) | -1000 | 10000

  init_param_float(PARAM_DIFF_PRESS_BIAS, "DIFF_PRESS_BIAS", 0.0f); // Differential Pressure Bias (Pa) | -10 | 10

  /************************/
  /*** RC CONFIGURATION ***/
  /************************/
  init_param_int(PARAM_RC_TYPE, "RC_TYPE", 0); // Type of RC input 0 - PPM, 1 - SBUS | 0 | 1
  init_param_int(PARAM_RC_X_CHANNEL, "RC_X_CHN", 0); // RC input channel mapped to x-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_Y_CHANNEL, "RC_Y_CHN", 1); // RC input channel mapped to y-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_Z_CHANNEL, "RC_Z_CHN", 3); // RC input channel mapped to z-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_F_CHANNEL, "RC_F_CHN", 2); // RC input channel mapped to F-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_F_AXIS, "RC_F_AXIS", 2);   // NED axis that RC F-channel gets mapped to 0 - X, 1 - Y, 2 - Z | 0 | 2
  init_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, "RC_ATT_OVRD_CHN", 4); // RC switch mapped to attitude override [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, "RC_THR_OVRD_CHN", 4); // RC switch channel mapped to throttle override [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,  "RC_ATT_CTRL_CHN",-1); // RC switch channel mapped to attitude control type [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_ARM_CHANNEL, "RC_ARM_CHN",-1); // RC switch channel mapped to arming (only if PARAM_ARM_STICKS is false) [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_NUM_CHANNELS, "RC_NUM_CHN", 6); // number of RC input channels | 1 | 8

  init_param_int(PARAM_RC_SWITCH_5_DIRECTION, "SWITCH_5_DIR", 1); // RC switch 5 toggle direction | -1 | 1
  init_param_int(PARAM_RC_SWITCH_6_DIRECTION, "SWITCH_6_DIR", 1); // RC switch 6 toggle direction | -1 | 1
  init_param_int(PARAM_RC_SWITCH_7_DIRECTION, "SWITCH_7_DIR", 1); // RC switch 7 toggle direction | -1 | 1
  init_param_int(PARAM_RC_SWITCH_8_DIRECTION, "SWITCH_8_DIR", 1); // RC switch 8 toggle direction | -1 | 1

  init_param_float(PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", 0.1); // RC stick deviation from center for override | 0.0 | 1.0
  init_param_int(PARAM_OVERRIDE_LAG_TIME, "RC_OVRD_LAG_T", 1000); // RC stick deviation lag time before returning control (ms) | 0 | 100000
  init_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "TAKE_MIN_THR", true); // Take minimum throttle between RC and computer at all times | 0 | 1
  init_param_float(PARAM_RC_MAX_THROTTLE, "RC_MAX_THR", 0.7f); // Maximum throttle command sent by full deflection of RC sticks, to maintain controllability during aggressive maneuvers | 0.0 | 1.0

  init_param_int(PARAM_RC_ATTITUDE_MODE, "RC_ATT_MODE", 1); // Attitude mode for RC sticks (0: rate, 1: angle). Overridden if RC_ATT_CTRL_CHN is set. | 0 | 1
  init_param_float(PARAM_RC_MAX_ROLL, "RC_MAX_ROLL", 0.786f); // Maximum roll angle command sent by full deflection of RC sticks | 0.0 | 3.14159
  init_param_float(PARAM_RC_MAX_PITCH, "RC_MAX_PITCH", 0.786f); // Maximum pitch angle command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  init_param_float(PARAM_RC_MAX_ROLLRATE, "RC_MAX_ROLLRATE", 3.14159f); // Maximum roll rate command sent by full stick deflection of RC sticks | 0.0 | 9.42477796077
  init_param_float(PARAM_RC_MAX_PITCHRATE, "RC_MAX_PITCHRATE", 3.14159f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  init_param_float(PARAM_RC_MAX_YAWRATE, "RC_MAX_YAWRATE", 1.507f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159

  /***************************/
  /*** FRAME CONFIGURATION ***/
  /***************************/
  init_param_int(PARAM_PRIMARY_MIXER, "PRIMARY_MIXER", Mixer::INVALID_MIXER); // Which mixer to choose for primary mixer - See Mixer documentation | 0 | 11
  init_param_int(PARAM_SECONDARY_MIXER, "SECONDARY_MIXER", Mixer::INVALID_MIXER); // Which mixer to choose for secondary mixer - See Mixer documentation | 0 | 11

  init_param_int(PARAM_FIXED_WING, "FIXED_WING", false); // switches on pass-through commands for fixed-wing operation | 0 | 1
  init_param_int(PARAM_ELEVATOR_REVERSE, "REV_ELEVATOR", 0); // reverses elevator servo output | 0 | 1
  init_param_int(PARAM_AILERON_REVERSE, "REV_AILERON", 0); // reverses aileron servo output | 0 | 1
  init_param_int(PARAM_RUDDER_REVERSE, "REV_RUDDER", 0); // reverses rudder servo output | 0 | 1

  init_param_float(PARAM_IMU_ROLL, "IMU_ROLL", 0.0f); // roll angle (deg) of IMU wrt aircraft body | 0 | 360
  init_param_float(PARAM_IMU_PITCH, "IMU_PITCH", 0.0f); // pitch angle (deg) of IMU wrt aircraft body | 0 | 360
  init_param_float(PARAM_IMU_YAW, "IMU_YAW", 0.0f); // yaw angle (deg) of IMU wrt aircraft body | 0 | 360

  init_param_float(PARAM_MAG_ROLL, "MAG_ROLL", 0.0f); // roll angle (deg) of magnetometer wrt aircraft body | 0 | 360
  init_param_float(PARAM_MAG_PITCH, "MAG_PITCH", 0.0f); // pitch angle (deg) of magnetometer wrt aircraft body | 0 | 360
  init_param_float(PARAM_MAG_YAW, "MAG_YAW", 0.0f); // yaw angle (deg) of magnetometer wrt aircraft body | 0 | 360

  /********************/
  /*** ARMING SETUP ***/
  /********************/
  init_param_float(PARAM_ARM_THRESHOLD, "RC_ARM_THRESHOLD", 0.15); // RC deviation from max/min in yaw and throttle for arming and disarming check (us) | 0 | 500

  /*****************************/
  /*** BATTERY MONITOR SETUP ***/
  /*****************************/
  init_param_float(PARAM_BATTERY_VOLTAGE_MULTIPLIER, "BATT_VOLT_MULT", 1.0f); // Multiplier for the voltage sensor | 0 | inf
  init_param_float(PARAM_BATTERY_CURRENT_MULTIPLIER, "BATT_CURR_MULT", 1.0f); // Multiplier for the current sensor | 0 | inf
  init_param_float(PARAM_BATTERY_VOLTAGE_ALPHA, "BATT_VOLT_LPF", 0.995f);   // Alpha value for the low pass filter on the reported battery voltage | 0 | 1
  init_param_float(PARAM_BATTERY_CURRENT_ALPHA, "BATT_CURR_LPF", 0.995f);   // Alpha value for the low pass filter on the reported battery current | 0 | 1

  /************************/
  /*** OFFBOARD CONTROL ***/
  /************************/
  init_param_int(PARAM_OFFBOARD_TIMEOUT, "OFFBOARD_TIMEOUT", 100); // Timeout in milliseconds for offboard commands, after which RC override is activated | 0 | 100000
}
// clang-format on

void Params::set_listeners(ParamListenerInterface * const listeners[], size_t num_listeners)
{
  listeners_ = listeners;
  num_listeners_ = num_listeners;
}

bool Params::read(void)
{
  if (!RF_.board_.memory_read(&params, sizeof(params_t))) { return false; }

  if (params.version != GIT_VERSION_HASH) { return false; }

  if (params.size != sizeof(params_t) || params.magic_be != 0xBE || params.magic_ef != 0xEF) {
    return false;
  }

  if (compute_checksum() != params.chk) { return false; }

  return true;
}

bool Params::write(void)
{
  params.version = GIT_VERSION_HASH;
  params.size = sizeof(params_t);
  params.magic_be = 0xBE;
  params.magic_ef = 0xEF;
  params.chk = compute_checksum();

  if (!RF_.board_.memory_write(&params, sizeof(params_t))) { return false; }
  return true;
}

void Params::change_callback(uint16_t id)
{
  // call the callback function for all listeners
  if (listeners_ != nullptr) {
    for (size_t i = 0; i < num_listeners_; i++) { listeners_[i]->param_change_callback(id); }
  }
}

uint16_t Params::lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (uint16_t id = 0; id < PARAMS_COUNT; id++) {
    bool match = true;
    for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++) {
      // compare each character
      if (name[i] != params.names[id][i]) {
        match = false;
        break;
      }

      // stop comparing if end of string is reached
      if (params.names[id][i] == '\0') { break; }
    }

    if (match) { return id; }
  }

  return PARAMS_COUNT;
}

bool Params::set_param_int(uint16_t id, int32_t value)
{
  if (id < PARAMS_COUNT && value != params.values[id].ivalue) {
    params.values[id].ivalue = value;
    change_callback(id);
    RF_.comm_manager_.send_param_value(id);
    return true;
  }
  return false;
}

bool Params::set_param_float(uint16_t id, float value)
{
  if (id < PARAMS_COUNT && value != params.values[id].fvalue) {
    params.values[id].fvalue = value;
    change_callback(id);
    RF_.comm_manager_.send_param_value(id);
    return true;
  }
  return false;
}

bool Params::set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  uint16_t id = lookup_param_id(name);
  return set_param_int(id, value);
}

bool Params::set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value)
{
  param_value_t tmp;
  tmp.fvalue = value;
  return set_param_by_name_int(name, tmp.ivalue);
}
} // namespace rosflight_firmware
