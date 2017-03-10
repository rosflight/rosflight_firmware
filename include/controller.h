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
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "mux.h"
#include "param.h"
#include "estimator.h"
#include "arming_fsm.h"

namespace rosflight {

class Controller
{
public:
  void run_controller();
  void init_controller(Arming_FSM*_fsm, Board* _board, //Mux* _mux, Mixer* _mixer,
                       Estimator* _estimator, Params* _params);
private:
  typedef struct
  {
    uint16_t kp_param_id;
    uint16_t ki_param_id;
    uint16_t kd_param_id;

    float *current_x;
    float *current_xdot;
    float *commanded_x;
    float *output;

    float max;
    float min;

    float integrator;
    float prev_x;
    float differentiator;
    float tau;
  } pid_t;

  pid_t pid_roll;
  pid_t pid_roll_rate;
  pid_t pid_pitch;
  pid_t pid_pitch_rate;
  pid_t pid_yaw_rate;
  pid_t pid_altitude;

  Estimator* estimator;
  Arming_FSM* fsm;
//  Mux* mux;
//  Mixer* mixer;
  Params* params;
  Board* board;
  CommLink* comm_link;

  void init_pid(pid_t *pid, uint16_t kp_param_id, uint16_t ki_param_id, uint16_t kd_param_id, float *current_x,
                float *current_xdot, float *commanded_x, float *output, float max, float min);
  void run_pid(pid_t *pid, float dt);

  float prev_time;

};
}
