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

#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wswitch"

#include <stdint.h>
#include "mavlink.h"

namespace rosflight {


// function definitions
void Mavlink::update_param(uint16_t param_id)
{
  if (param_id < PARAMS_COUNT)
  {
    MAV_PARAM_TYPE type;
    switch (params_->get_param_type(param_id))
    {
    case PARAM_TYPE_INT32:
      type = MAV_PARAM_TYPE_INT32;
      break;
    case PARAM_TYPE_FLOAT:
      type = MAV_PARAM_TYPE_REAL32;
      break;
    default:
      return;
    }

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(params_->get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                                 params_->get_param_name(param_id), params_->get_param_float(param_id), type, PARAMS_COUNT, param_id);
    send_message(msg);
  }
}

void Mavlink::mavlink_handle_msg_param_request_list(void)
{
  send_params_index = 0;
}

void Mavlink::mavlink_handle_msg_param_request_read(const mavlink_message_t *const msg)
{
  mavlink_param_request_read_t read;
  mavlink_msg_param_request_read_decode(msg, &read);

  if (read.target_system == (uint8_t) params_->get_param_int(PARAM_SYSTEM_ID)) // TODO check if component id matches?
  {
    uint16_t id = (read.param_index < 0) ? params_->lookup_param_id(read.param_id) : (uint16_t) read.param_index;

    if (id < PARAMS_COUNT)
      update_param(id);
  }
}

void Mavlink::mavlink_handle_msg_param_set(const mavlink_message_t *const msg)
{
  mavlink_param_set_t set;
  mavlink_msg_param_set_decode(msg, &set);

  if (set.target_system == (uint8_t) params_->get_param_int(PARAM_SYSTEM_ID)) // TODO check if component id matches?
  {
    uint16_t id = params_->lookup_param_id(set.param_id);

    if (id < PARAMS_COUNT)
    {
      param_type_t candidate_type;
      switch (set.param_type)
      {
      case MAV_PARAM_TYPE_INT32:
        candidate_type = PARAM_TYPE_INT32;
        break;
      case MAV_PARAM_TYPE_REAL32:
        candidate_type = PARAM_TYPE_FLOAT;
        break;
      default:
        candidate_type = PARAM_TYPE_INVALID;
        break;
      }

      if (candidate_type == params_->get_param_type(id))
      {
        switch (candidate_type)
        {
        case PARAM_TYPE_INT32:
          params_->set_param_int(id, *(int32_t *) &set.param_value);
          break;
        case PARAM_TYPE_FLOAT:
          params_->set_param_float(id, set.param_value);
          break;
        }
      }
    }
  }
}

void Mavlink::mavlink_send_next_param(void)
{
  if (send_params_index < PARAMS_COUNT)
  {
    update_param((uint16_t) send_params_index);
    send_params_index++;
  }
}

}
