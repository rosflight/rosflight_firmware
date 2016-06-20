#include <stdbool.h>
#include <stdint.h>

#include "flash.h"
#include "mavlink.h"
#include "mavlink_stream.h"

#include "param.h"

//TODO temporary
#include <stdio.h>

// global variable definitions
params_t _params;

// local function definitions
static void init_param(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  _params.values[id] = value;
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
}

// function definitions
void init_params(void)
{
  initEEPROM();
  if (!read_params())
  {
    set_param_defaults();
    write_params();
  }

  for (param_id_t id = 0; id < PARAMS_COUNT; id++)
    param_change_callback(id);
}

void set_param_defaults(void)
{
  // temporary: replace with actual initialisation of rest of params
  char temp_name[PARAMS_NAME_LENGTH];
  for (param_id_t id = 0; id < PARAMS_COUNT; id++)
  {
    sprintf(temp_name, "TEMP_%c%c", 'A' + id/10, 'A' + id%10);
    init_param(id, temp_name, id);
  }
  init_param(PARAM_BOARD_REVISION, "BOARD_REV", 5);

  init_param(PARAM_SYSTEM_ID, "SYS_ID", 1);
  init_param(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1);
  init_param(PARAM_STREAM_IMU_RATE, "STRM_IMU", 100);
  init_param(PARAM_STREAM_SERVO_OUTPUT_RAW_RATE, "STRM_SERVO", 50);
  init_param(PARAM_STREAM_RC_RAW_RATE, "STRM_RC", 50);

  init_param(PARAM_GYRO_LPF_ALPHA, "GYRO_LPF_ALPHA", 100);
  init_param(PARAM_ACC_LPF_ALPHA, "GYRO_LPF_ALPHA", 500);

  init_param(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0);
  init_param(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0);
  init_param(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0);
  init_param(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0);
  init_param(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0);
  init_param(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0);

  init_param(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_PERIOD", 50);
  init_param(PARAM_MOTOR_IDLE_PWM, "MOTOR_IDLE_PWM", 1100);
  init_param(PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", true);
  init_param(PARAM_RC_TYPE, "RC_TYPE", 1);
  init_param(PARAM_RC_X_CHANNEL, "RC_X_CHN", 0);
  init_param(PARAM_RC_Y_CHANNEL, "RC_Y_CHN", 1);
  init_param(PARAM_RC_Z_CHANNEL, "RC_Z_CHN", 3);
  init_param(PARAM_RC_F_CHANNEL, "RC_F_CHN", 2);

  init_param(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, "RC_ATT_OVRD_CHN", 4);
  init_param(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, "RC_THR_OVRD_CHN", 4);
  init_param(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,  "RC_ATT_CTRL_CHN", 6);
  init_param(PARAM_RC_F_CONTROL_TYPE_CHANNEL,    "RC_F_CTRL_CHN", 6);

  init_param(PARAM_RC_X_CENTER, "RC_X_CENTER", 1500);
  init_param(PARAM_RC_Y_CENTER, "RC_Y_CENTER", 1500);
  init_param(PARAM_RC_Z_CENTER, "RC_Z_CENTER", 1500);
  init_param(PARAM_RC_F_BOTTOM, "RC_F_BOTTOM", 1000);
  init_param(PARAM_RC_X_RANGE,  "RC_X_RANGE", 1000);
  init_param(PARAM_RC_Y_RANGE,  "RC_Y_RANGE", 1000);
  init_param(PARAM_RC_Z_RANGE,  "RC_Z_RANGE", 1000);
  init_param(PARAM_RC_F_RANGE,  "RC_F_RANGE", 1000);

  init_param(PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", 100);
  init_param(PARAM_OVERRIDE_LAG_TIME, "OVRD_LAG_TIME", 1000);
  init_param(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "MIN_THROTTLE", false);

  init_param(PARAM_RC_MAX_ROLL_MRAD, "RC_MAX_ROLL", 786); // 45 deg
  init_param(PARAM_RC_MAX_PITCH_MRAD, "RC_MAX_PITCH", 786);
  init_param(PARAM_RC_MAX_ROLLRATE_MRAD_S, "RC_MAX_ROLLRATE", 12566); // 720 deg/s
  init_param(PARAM_RC_MAX_PITCHRATE_MRAD_S, "RC_MAX_PITCHRATE", 12566); // 720 deg/s
  init_param(PARAM_RC_MAX_YAWRATE_MRAD_S, "RC_MAX_YAWRATE", 6283); // 360 deg/s

  init_param(PARAM_ARM_STICKS, "ARM_STICKS", false);
  init_param(PARAM_ARM_CHANNEL, "ARM_CHANNEL", 5);
  init_param(PARAM_ARM_THRESHOLD, "ARM_THRESHOLD", 400);

  init_param(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1000);

  init_param(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", 400);
  init_param(PARAM_MAX_ROLL_RATE, "MAX_ROLL_RATE", 12566);

  init_param(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", 400);
  init_param(PARAM_MAX_PITCH_RATE, "MAX_PITCH_RATE", 12566);

  init_param(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", 0);
  init_param(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", 0);
  init_param(PARAM_MAX_YAW_RATE, "MAX_YAW_RATE", 6283);

  init_param(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", 2000);
  init_param(PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", 0);
  init_param(PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", 0);
  init_param(PARAM_MAX_ROLL_ANGLE, "MAX_ROLL_ANG", 786);

  init_param(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", 2000);
  init_param(PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", 0);
  init_param(PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", 0);
  init_param(PARAM_MAX_PITCH_ANGLE, "MAX_PITCH_ANG", 786);

  init_param(PARAM_PID_ALT_P, "PID_ALT_P", 10);
  init_param(PARAM_PID_ALT_I, "PID_ALT_I", 0);
  init_param(PARAM_PID_ALT_D, "PID_ALT_D", 0);


  init_param(PARAM_MIXER, "MIXER", 0);
  init_param(PARAM_ELEVATOR_REVERSE, "ELEVATOR_REV", 1);
  init_param(PARAM_AILERON_REVERSE, "AIL_REV", 0);
  init_param(PARAM_RUDDER_REVERSE, "RUDDER_REV", 0);
  init_param(PARAM_FIXED_WING, "FIXED_WING", true);
}

bool read_params(void)
{
  return readEEPROM();
}

bool write_params(void)
{
  return writeEEPROM(true);
}

void param_change_callback(param_id_t id)
{
  switch (id)
  {
  case PARAM_SYSTEM_ID:
    mavlink_system.sysid = _params.values[PARAM_SYSTEM_ID];
    break;
  case PARAM_STREAM_HEARTBEAT_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_HEARTBEAT, _params.values[PARAM_STREAM_HEARTBEAT_RATE]);
    break;
  case PARAM_STREAM_IMU_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_IMU, _params.values[PARAM_STREAM_IMU_RATE]);
    break;
  case PARAM_STREAM_SERVO_OUTPUT_RAW_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW, _params.values[PARAM_STREAM_SERVO_OUTPUT_RAW_RATE]);
    break;
  case PARAM_STREAM_RC_RAW_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_RC_RAW, _params.values[PARAM_STREAM_RC_RAW_RATE]);
  default:
    // no action needed for this parameter
    break;
  }
}

param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (param_id_t id = 0; id < PARAMS_COUNT; id++)
  {
    bool match = true;
    for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++)
    {
      // compare each character
      if (name[i] != _params.names[id][i])
      {
        match = false;
        break;
      }

      // stop comparing if end of string is reached
      if (_params.names[id][i] == '\0')
        break;
    }

    if (match)
      return id;
  }

  return PARAMS_COUNT;
}
