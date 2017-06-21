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
#include <stdint.h>

#include "mavlink.h"
#include "rosflight.h"

namespace rosflight_firmware {

Mavlink::Mavlink()
{
  initialized = false;
}

// function definitions
void Mavlink::init(ROSflight *firmware)
{
  RF_ = firmware;
  RF_->board_->serial_init(RF_->params_.get_param_int(PARAM_BAUD_RATE));

  sysid = RF_->params_.get_param_int(PARAM_SYSTEM_ID);
  compid = 250;

  _offboard_control_time = 0;
  send_params_index = PARAMS_COUNT;

  // Register Param change callbacks
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_HEARTBEAT, std::placeholders::_1), PARAM_STREAM_HEARTBEAT_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_ATTITUDE, std::placeholders::_1), PARAM_STREAM_ATTITUDE_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_IMU, std::placeholders::_1), PARAM_STREAM_IMU_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_ATTITUDE, std::placeholders::_1), PARAM_STREAM_ATTITUDE_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_DIFF_PRESSURE, std::placeholders::_1), PARAM_STREAM_AIRSPEED_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_BARO, std::placeholders::_1), PARAM_STREAM_BARO_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_SONAR, std::placeholders::_1), PARAM_STREAM_SONAR_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_MAG, std::placeholders::_1), PARAM_STREAM_MAG_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_SERVO_OUTPUT_RAW, std::placeholders::_1), PARAM_STREAM_OUTPUT_RAW_RATE);
  RF_->params_.add_callback(std::bind(&Mavlink::set_streaming_rate, this, STREAM_ID_RC_RAW, std::placeholders::_1), PARAM_STREAM_RC_RAW_RATE);

  initialized = true;
}

void Mavlink::send_message(const mavlink_message_t &msg)
{
  if (initialized)
  {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(data, &msg);
    for (int i = 0; i < len; i++)
    {
      RF_->board_->serial_write(data[i]);
    }
  }
}

}

