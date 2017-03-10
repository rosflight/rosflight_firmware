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

#include <stdbool.h>

#include "board.h"
#include "mavlink.h"
#include "mixer.h"
#include "sensors.h"
#include "estimator.h"
#include "param.h"
#include "arming_fsm.h"
#include "rc.h"
#include "mavlink_log.h"

namespace rosflight
{

// local variable definitions
static mavlink_stream_t mavlink_streams[MAVLINK_STREAM_COUNT] =
{
  { .period_us = 0, .next_time_us = 0, .send_function = mavlink_send_heartbeat },
  { .period_us = 0, .next_time_us = 0, .send_function = mavlink_send_status},

  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_attitude },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_imu },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_diff_pressure },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_baro },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_sonar },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_mag },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_rosflight_output_raw },
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_rc_raw },

  { .period_us = 5000,   .next_time_us = 0, .send_function = mavlink_send_low_priority }
};

// local function definitions
 void Mavlink::mavlink_send_heartbeat(void)
{
  MAV_MODE armed_mode = MAV_MODE_ENUM_END; // used for failsafe
//  if (_armed_state == ARMED)
//    armed_mode = MAV_MODE_MANUAL_ARMED;
//  else if (_armed_state == DISARMED)
//    armed_mode = MAV_MODE_MANUAL_DISARMED;

  uint8_t control_mode = 0;
//  if (params_->get_param_int(PARAM_FIXED_WING))
//  {
//    control_mode = MODE_PASS_THROUGH;
//  }
//  else if (rc_switch(params_->get_param_int(PARAM_RC_F_CONTROL_TYPE_CHANNEL)))
//  {
//    control_mode = MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
//  }
//  else
//  {
//    control_mode = rc_switch(params_->get_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL)) ? MODE_ROLL_PITCH_YAWRATE_THROTTLE :
//                   MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
//  }

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(params_->get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                             params_->get_param_int(PARAM_FIXED_WING) ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_GENERIC,
                             armed_mode,
                             control_mode,
                             MAV_STATE_STANDBY);
  send_message(msg);
}

 void Mavlink::mavlink_send_attitude(void)
{
  mavlink_message_t msg;
//  mavlink_msg_attitude_quaternion_pack(sysid, compid, &msg,
//                                        board_->clock_millis(),
//                                        _current_state.q.w,
//                                        _current_state.q.x,
//                                        _current_state.q.y,
//                                        _current_state.q.z,
//                                        _current_state.omega.x,
//                                        _current_state.omega.y,
//                                        _current_state.omega.z);
  send_message(msg);
}

void Mavlink::mavlink_send_imu(void)
{
  mavlink_message_t msg;
//  mavlink_msg_small_imu_pack(sysid, compid, &msg,
//                             _imu_time,
//                             _accel.x,
//                             _accel.y,
//                             _accel.z,
//                             _gyro.x,
//                             _gyro.y,
//                             _gyro.z,
//                             _imu_temperature);
  send_message(msg);

}

void Mavlink::mavlink_send_servo_output_raw(void)
{
  mavlink_message_t msg;
//  mavlink_msg_servo_output_raw_pack(sysid, compid, &msg,
//                                    board_->clock_micros(),
//                                    0,
//                                    _outputs[0],
//                                    _outputs[1],
//                                    _outputs[2],
//                                    _outputs[3],
//                                    _outputs[4],
//                                    _outputs[5],
//                                    _outputs[6],
//                                    _outputs[7]);
  send_message(msg);
}

 void Mavlink::mavlink_send_rc_raw(void)
{
  mavlink_message_t msg;
//  mavlink_msg_rc_channels_pack(sysid, compid, &msg,
//                               board_->clock_millis(),
//                               0,
//                               board_->pwm_read(0),
//                               board_->pwm_read(1),
//                               board_->pwm_read(2),
//                               board_->pwm_read(3),
//                               board_->pwm_read(4),
//                               board_->pwm_read(5),
//                               board_->pwm_read(6),
//                               board_->pwm_read(7),
//                               0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0);
  send_message(msg);
}

void Mavlink::mavlink_send_diff_pressure(void)
{
  if (board_->diff_pressure_present())
  {
    mavlink_message_t msg;
//    mavlink_msg_diff_pressure_pack(sysid, compid, &msg,
//                                   _diff_pressure_velocity,
//                                   _diff_pressure,
//                                   _diff_pressure_temp);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_baro(void)
{
  if (board_->baro_present())
  {
    mavlink_message_t msg;
//    mavlink_msg_small_baro_pack(sysid, compid, &msg,
//                                _baro_altitude,
//                                _baro_pressure,
//                                _baro_temperature);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_sonar(void)
{
  if (board_->sonar_present())
  {
    mavlink_message_t msg;
//    mavlink_msg_small_sonar_pack(sysid, compid, &msg,
//                                 _sonar_range,
//                                 8.0,
//                                 0.25);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_mag(void)
{
  if (board_->mag_present())
  {
    mavlink_message_t msg;
//    mavlink_msg_small_mag_pack(sysid, compid, &msg,
//                               _mag.x,
//                               _mag.y,
//                               _mag.z);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}

// function definitions
void Mavlink::stream()
{
  uint64_t time_us = board_->clock_micros();
  for (int i = 0; i < STREAM_COUNT; i++)
  {
    if (time_us >= mavlink_streams[i].next_time_us)
    {
      // if we took too long, set the last_time_us to be where it should have been
      mavlink_streams[i].last_time_us += mavlink_streams[i].period_us;
      (this->*mavlink_streams[i].send_function)();
    }
  }
}

void Mavlink::set_streaming_rate(uint8_t stream_id, int32_t rate)
{
  mavlink_streams[stream_id].period_us = (rate == 0 ? 0 : 1000000/rate);
}

void Mavlink::mavlink_stream_set_period(uint8_t stream_id, uint32_t period_us)
{
  mavlink_streams[stream_id].period_us = period_us;
}

}
