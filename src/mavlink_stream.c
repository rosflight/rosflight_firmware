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
#include "mavlink_param.h"
#include "mixer.h"
#include "sensors.h"
#include "estimator.h"
#include "param.h"
#include "mode.h"
#include "rc.h"
#include "mode.h"

#include "mavlink_stream.h"
#include "mavlink_util.h"
#include "mavlink_log.h"

uint32_t _loop_time_us;

// Declarations of local function definitions
static void mavlink_send_heartbeat(void);
static void mavlink_send_status(void);
static void mavlink_send_attitude(void);
static void mavlink_send_imu(void);
static void mavlink_send_rosflight_output_raw(void);
static void mavlink_send_rc_raw(void);
static void mavlink_send_diff_pressure(void);
static void mavlink_send_baro(void);
static void mavlink_send_sonar(void);
static void mavlink_send_mag(void);
static void mavlink_send_lidar(void);
static void mavlink_send_low_priority(void);

// typedefs
typedef struct
{
  uint32_t period_us;
  uint64_t next_time_us;
  void (*send_function)(void);
} mavlink_stream_t;

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
  { .period_us = 0,  .next_time_us = 0, .send_function = mavlink_send_lidar },

  { .period_us = 5000,   .next_time_us = 0, .send_function = mavlink_send_low_priority }
};

// local function definitions
static void mavlink_send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
                             get_param_int(PARAM_FIXED_WING) ? MAV_TYPE_FIXED_WING : MAV_TYPE_QUADROTOR,
                             0, 0, 0, 0);
}

// local function definitions
static void mavlink_send_status(void)
{
  volatile uint8_t status = 0;
  status |= (_armed_state & ARMED) ? ROSFLIGHT_STATUS_ARMED : 0x00;
  status |= (_armed_state & FAILSAFE) ? ROSFLIGHT_STATUS_IN_FAILSAFE : 0x00;
  status |= (rc_override_active()) ? ROSFLIGHT_STATUS_RC_OVERRIDE : 0x00;
  status |= (offboard_control_active()) ? ROSFLIGHT_STATUS_OFFBOARD_CONTROL_ACTIVE : 0x00;

  uint8_t control_mode = 0;
  if (get_param_int(PARAM_FIXED_WING))
    control_mode = MODE_PASS_THROUGH;
  else
    // TODO - Support rate mode
    control_mode = MODE_ROLL_PITCH_YAWRATE_THROTTLE;



  mavlink_msg_rosflight_status_send(MAVLINK_COMM_0,
                                    status,
                                    _error_state,
                                    control_mode,
                                    num_sensor_errors(),
                                    _loop_time_us);
}

static void mavlink_send_attitude(void)
{
  mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0,
                                       clock_millis(),
                                       _current_state.q.w,
                                       _current_state.q.x,
                                       _current_state.q.y,
                                       _current_state.q.z,
                                       _current_state.omega.x,
                                       _current_state.omega.y,
                                       _current_state.omega.z);
}

static void mavlink_send_imu(void)
{
  // If we haven't sent this IMU measurement yet, send it
  if (!_imu_sent)
  {
    mavlink_msg_small_imu_send(MAVLINK_COMM_0,
                               _imu_time,
                               _accel.x,
                               _accel.y,
                               _accel.z,
                               _gyro.x,
                               _gyro.y,
                               _gyro.z,
                               _imu_temperature);
  }
  else
  {
    // Otherwise, wait and signal that we still need to send IMU
    mavlink_streams[MAVLINK_STREAM_ID_IMU].next_time_us -= mavlink_streams[MAVLINK_STREAM_ID_IMU].period_us;
  }
}

static void mavlink_send_rosflight_output_raw(void)
{
  mavlink_msg_rosflight_output_raw_send(MAVLINK_COMM_0,
                                        clock_millis(),
                                        _outputs);
}

static void mavlink_send_rc_raw(void)
{
  mavlink_msg_rc_channels_send(MAVLINK_COMM_0,
                               clock_millis(),
                               0,
                               pwm_read(0),
                               pwm_read(1),
                               pwm_read(2),
                               pwm_read(3),
                               pwm_read(4),
                               pwm_read(5),
                               pwm_read(6),
                               pwm_read(7),
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0);
}

static void mavlink_send_diff_pressure(void)
{
  if (diff_pressure_present())
  {
    mavlink_msg_diff_pressure_send(MAVLINK_COMM_0, _diff_pressure_velocity, _diff_pressure, _diff_pressure_temp);
  }
}

static void mavlink_send_baro(void)
{
  if (baro_present())
  {
    mavlink_msg_small_baro_send(MAVLINK_COMM_0, _baro_altitude, _baro_pressure, _baro_temperature);
  }
}

static void mavlink_send_sonar(void)
{
  if (sonar_present())
  {
    mavlink_msg_small_sonar_send(MAVLINK_COMM_0,
                                 _sonar_range,
                                 8.0,
                                 0.25);
  }
}

static void mavlink_send_mag(void)
{
  if (mag_present())
  {
    mavlink_msg_small_mag_send(MAVLINK_COMM_0,
                               _mag.x,
                               _mag.y,
                               _mag.z);
  }
}

static void mavlink_send_lidar(void) 
{
  if (lidar_present())
  {
    mavlink_msg_lidar_send(MAVLINK_COMM_0, _lidar_altitude);
  }
}

static void mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}

// function definitions
void mavlink_stream(uint64_t time_us)
{
  for (int i = 0; i < MAVLINK_STREAM_COUNT; i++)
  {
    if (time_us >= mavlink_streams[i].next_time_us)
    {
      mavlink_streams[i].next_time_us += mavlink_streams[i].period_us;
      mavlink_streams[i].send_function();
    }
  }
}

void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate)
{
  mavlink_streams[stream_id].period_us = (rate == 0 ? 0 : 1000000/rate);
}

void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us)
{
  mavlink_streams[stream_id].period_us = period_us;
}
