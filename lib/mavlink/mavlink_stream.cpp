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

#include "rosflight.h"

namespace rosflight_firmware
{

void Mavlink::send_log_message(uint8_t severity, char *text)
{
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                              severity,
                              text);
  send_message(msg);
}


void Mavlink::mavlink_send_heartbeat(void)
{
    uint8_t control_mode = 0;
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                             RF_->params_.get_param_int(PARAM_FIXED_WING) ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
                             0, 0, 0, 0);
  send_message(msg);
}

void Mavlink::mavlink_send_status(void)
{
  volatile uint8_t status = 0;
  status |= (RF_->state_manager_.state().armed) ? ROSFLIGHT_STATUS_ARMED : 0x00;
  status |= (RF_->state_manager_.state().failsafe) ? ROSFLIGHT_STATUS_IN_FAILSAFE : 0x00;
  status |= (RF_->mux_.rc_override_active()) ? ROSFLIGHT_STATUS_RC_OVERRIDE : 0x00;
  status |= (RF_->mux_.offboard_control_active()) ? ROSFLIGHT_STATUS_OFFBOARD_CONTROL_ACTIVE : 0x00;

  uint8_t control_mode = 0;
  if (RF_->params_.get_param_int(PARAM_FIXED_WING))
    control_mode = MODE_PASS_THROUGH;
  else if (RF_->mux_._combined_control.x.type == ANGLE)
    control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  else
    control_mode = MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;

  mavlink_message_t msg;
  mavlink_msg_rosflight_status_pack(RF_->params_.get_param_int(PARAM_SYSTEM_ID), 0, &msg,
                                    status,
                                    RF_->state_manager_.state().error_codes,
                                    control_mode,
                                    RF_->board_.num_sensor_errors(),
                                    RF_->get_loop_time_us());
  send_message(msg);
}


void Mavlink::mavlink_send_attitude(void)
{
  mavlink_message_t msg;
  mavlink_msg_attitude_quaternion_pack(sysid_, compid_, &msg,
                                       RF_->estimator_.get_estimator_timestamp()/1000,
                                       RF_->estimator_.get_attitude().w,
                                       RF_->estimator_.get_attitude().x,
                                       RF_->estimator_.get_attitude().y,
                                       RF_->estimator_.get_attitude().z,
                                       RF_->estimator_.get_angular_velocity().x,
                                       RF_->estimator_.get_angular_velocity().y,
                                       RF_->estimator_.get_angular_velocity().z);
  send_message(msg);
}

void Mavlink::mavlink_send_imu(void)
{
  if(RF_->sensors_.should_send_imu_data())
  {
    mavlink_message_t msg;
    vector_t accel = RF_->sensors_.data()._accel;
    vector_t gyro = RF_->sensors_.data()._gyro;
    mavlink_msg_small_imu_pack(sysid_, compid_, &msg,
                               RF_->sensors_.data()._imu_time,
                               accel.x,
                               accel.y,
                               accel.z,
                               gyro.x,
                               gyro.y,
                               gyro.z,
                               RF_->sensors_.data()._imu_temperature);
    send_message(msg);
  }
  else
  {
    // Otherwise, wait and signal that we still need to send IMU
    mavlink_streams_[STREAM_ID_IMU].next_time_us -= mavlink_streams_[STREAM_ID_IMU].period_us;
  }

}

void Mavlink::mavlink_send_output_raw(void)
{
  mavlink_message_t msg;
    mavlink_msg_rosflight_output_raw_pack(sysid_, compid_, &msg,
                                      RF_->board_.clock_millis(),
                                      RF_->mixer_._outputs);
    send_message(msg);
}

void Mavlink::mavlink_send_rc_raw(void)
{
  mavlink_message_t msg;
  mavlink_msg_rc_channels_pack(sysid_, compid_, &msg,
                               RF_->board_.clock_millis(),
                               0,
                               RF_->board_.pwm_read(0),
                               RF_->board_.pwm_read(1),
                               RF_->board_.pwm_read(2),
                               RF_->board_.pwm_read(3),
                               RF_->board_.pwm_read(4),
                               RF_->board_.pwm_read(5),
                               RF_->board_.pwm_read(6),
                               RF_->board_.pwm_read(7),
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0);
  send_message(msg);
}

void Mavlink::mavlink_send_diff_pressure(void)
{
  if (RF_->board_.diff_pressure_present())
  {
    mavlink_message_t msg;
    mavlink_msg_diff_pressure_pack(sysid_, compid_, &msg,
                                   RF_->sensors_.data()._diff_pressure_velocity,
                                   RF_->sensors_.data()._diff_pressure,
                                   RF_->sensors_.data()._diff_pressure_temp);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_baro(void)
{
  if (RF_->board_.baro_present())
  {
    mavlink_message_t msg;
    mavlink_msg_small_baro_pack(sysid_, compid_, &msg,
                                RF_->sensors_.data()._baro_altitude,
                                RF_->sensors_.data()._baro_pressure,
                                RF_->sensors_.data()._baro_temperature);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_sonar(void)
{
  if (RF_->board_.sonar_present())
  {
    mavlink_message_t msg;
    mavlink_msg_small_sonar_pack(sysid_, compid_, &msg,
                                 RF_->sensors_.data()._sonar_range,
                                 8.0,
                                 0.25);
    send_message(msg);
  }
}

void Mavlink::mavlink_send_mag(void)
{
  if (RF_->board_.mag_present())
  {
    mavlink_message_t msg;
    mavlink_msg_small_mag_pack(sysid_, compid_, &msg,
                               RF_->sensors_.data()._mag.x,
                               RF_->sensors_.data()._mag.y,
                               RF_->sensors_.data()._mag.z);
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
  uint64_t time_us = RF_->board_.clock_micros();
  for (int i = 0; i < STREAM_COUNT; i++)
  {
    if (time_us >= mavlink_streams_[i].next_time_us)
    {
      // if we took too long, set the last_time_us to be where it should have been
      mavlink_streams_[i].next_time_us += mavlink_streams_[i].period_us;
      (this->*mavlink_streams_[i].send_function)();
    }
  }
}

void Mavlink::set_streaming_rate(uint8_t stream_id, int16_t param_id)
{
  mavlink_streams_[stream_id].period_us = (RF_->params_.get_param_int(param_id) == 0 ? 0 : 1000000/RF_->params_.get_param_int(param_id));
}

void Mavlink::mavlink_stream_set_period(uint8_t stream_id, uint32_t period_us)
{
  mavlink_streams_[stream_id].period_us = period_us;
}

}
