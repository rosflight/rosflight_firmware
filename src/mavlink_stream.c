#include <stdbool.h>

#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mixer.h"
#include "sensors.h"
#include "estimator.h"
#include "param.h"

#include "mavlink_stream.h"
#include "mavlink_util.h"
#include "mavlink_log.h"

// typedefs
typedef struct
{
  uint32_t period_us;
  uint32_t last_time_us;
  void (*send_function)(void);
} mavlink_stream_t;

// local function definitions
static void mavlink_send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0,
                             MAV_STATE_STANDBY);
}

static void mavlink_send_attitude(void)
{
  mavlink_msg_attitude_send(MAVLINK_COMM_0,
                            millis(),
                            _current_state.phi,
                            _current_state.theta,
                            _current_state.psi,
                            _current_state.p,
                            _current_state.q,
                            _current_state.r);
}

static void mavlink_send_imu(void)
{
  if (_params.values[PARAM_STREAM_ADJUSTED_GYRO])
  {
    mavlink_msg_small_imu_send(MAVLINK_COMM_0,
                               _imu_time,
                               _accel.x,
                               _accel.y,
                               _accel.z,
                               _gyro.x - _adaptive_gyro_bias.x,
                               _gyro.y - _adaptive_gyro_bias.y,
                               _gyro.z - _adaptive_gyro_bias.z,
                               _imu_temperature);
  }
  else
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
}

static void mavlink_send_servo_output_raw(void)
{
  mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0,
                                    micros(),
                                    0,
                                    _outputs[0],
                                    _outputs[1],
                                    _outputs[2],
                                    _outputs[3],
                                    _outputs[4],
                                    _outputs[5],
                                    _outputs[6],
                                    _outputs[7]);
}

static void mavlink_send_rc_raw(void)
{
  mavlink_msg_rc_channels_send(MAVLINK_COMM_0,
                               millis(),
                               0,
                               pwmRead(0),
                               pwmRead(1),
                               pwmRead(2),
                               pwmRead(3),
                               pwmRead(4),
                               pwmRead(5),
                               pwmRead(6),
                               pwmRead(7),
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0);
}

static void mavlink_send_diff_pressure(void)
{
  if (_diff_pressure_present)
  {
    mavlink_msg_diff_pressure_send(MAVLINK_COMM_0, _diff_pressure, _temperature);
  }
}

static void mavlink_send_baro(void)
{
  if (_baro_present)
  {
    mavlink_msg_small_baro_send(MAVLINK_COMM_0, _baro_pressure, _temperature);
  }
}

static void mavlink_send_sonar(void)
{
  if (_sonar_present)
  {
    mavlink_msg_distance_sensor_send(MAVLINK_COMM_0,
                                     _sonar_time,
                                     24,
                                     822,
                                     _sonar_range,
                                     MAV_DISTANCE_SENSOR_ULTRASOUND,
                                     1,
                                     MAV_SENSOR_ROTATION_PITCH_180,
                                     1);
  }
}

static void mavlink_send_low_priority(void)
{
  mavlink_send_next_param();
}

// local variable definitions
static mavlink_stream_t mavlink_streams[MAVLINK_STREAM_COUNT] =
{
  { .period_us = 1000000, .last_time_us = 0, .send_function = mavlink_send_heartbeat },

  { .period_us = 200000,  .last_time_us = 0, .send_function = mavlink_send_attitude },

  { .period_us = 1000,    .last_time_us = 0, .send_function = mavlink_send_imu },
  { .period_us = 200000,  .last_time_us = 0, .send_function = mavlink_send_diff_pressure },
  { .period_us = 200000,  .last_time_us = 0, .send_function = mavlink_send_baro },
  { .period_us = 100000,  .last_time_us = 0, .send_function = mavlink_send_sonar },

  { .period_us = 0,       .last_time_us = 0, .send_function = mavlink_send_servo_output_raw },
  { .period_us = 0,       .last_time_us = 0, .send_function = mavlink_send_rc_raw },
  { .period_us = 10000,   .last_time_us = 0, .send_function = mavlink_send_low_priority }
};

// function definitions
void mavlink_stream(uint32_t time_us)
{
  for (int i = 0; i < MAVLINK_STREAM_COUNT; i++)
  {
    if (mavlink_streams[i].period_us && time_us - mavlink_streams[i].last_time_us >= mavlink_streams[i].period_us)
    {
      // if we took too long, set the last_time_us to be where it should have been
      mavlink_streams[i].last_time_us += mavlink_streams[i].period_us;
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
