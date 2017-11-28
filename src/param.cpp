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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "mixer.h"

#include "param.h"

#include "rosflight.h"

namespace rosflight_firmware
{

Params::Params(ROSflight& _rf) :
  RF_(_rf)
{
  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
    callbacks[id] = NULL;
}

// local function definitions
void Params::init_param_int(uint16_t id, const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  memcpy(params.names[id], name, PARAMS_NAME_LENGTH);
  params.values[id].ivalue = value;
  params.types[id] = PARAM_TYPE_INT32;
}

void Params::init_param_float(uint16_t id, const char name[PARAMS_NAME_LENGTH], float value)
{
  memcpy(params.names[id], name, PARAMS_NAME_LENGTH);
  params.values[id].fvalue = value;
  params.types[id] = PARAM_TYPE_FLOAT;
}

uint8_t Params::compute_checksum(void)
{
  uint8_t chk = 0;
  const char *p;

  for (p = reinterpret_cast<const char *>(&params.values); p < reinterpret_cast<const char *>(&params.values) + 4*PARAMS_COUNT; p++)
    chk ^= *p;
  for (p = reinterpret_cast<const char *>(&params.names); p < reinterpret_cast<const char *>(&params.names) + PARAMS_COUNT*PARAMS_NAME_LENGTH; p++)
    chk ^= *p;
  for (p = reinterpret_cast<const char *>(&params.types); p < reinterpret_cast<const char *>(&params.types) + PARAMS_COUNT; p++)
    chk ^= *p;

  return chk;
}


// function definitions
void Params::init()
{
  RF_.board_.memory_init();
  if (!read())
  {
    set_defaults();
    write();
  }
}

void Params::set_defaults(void)
{
  /******************************/
  /*** HARDWARE CONFIGURATION ***/
  /******************************/
  init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600); // Baud rate of MAVlink communication with onboard computer | 9600 | 921600

  /*****************************/
  /*** MAVLINK CONFIGURATION ***/
  /*****************************/
  init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1); // Mavlink System ID  | 1 | 255
  init_param_int(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1); // Rate of heartbeat streaming (Hz) | 0 | 1000
  init_param_int(PARAM_STREAM_STATUS_RATE, "STRM_STATUS", 10); // Rate of status streaming (Hz) | 0 | 1000

  init_param_int(PARAM_STREAM_ATTITUDE_RATE, "STRM_ATTITUDE", 200); // Rate of attitude stream (Hz) | 0 | 1000
  init_param_int(PARAM_STREAM_IMU_RATE, "STRM_IMU", 500); // Rate of IMU stream (Hz) | 0 | 1000
  init_param_int(PARAM_STREAM_MAG_RATE, "STRM_MAG", 50); // Rate of magnetometer stream (Hz) | 0 | 75
  init_param_int(PARAM_STREAM_BARO_RATE, "STRM_BARO", 50); // Rate of barometer stream (Hz) | 0 | 100
  init_param_int(PARAM_STREAM_AIRSPEED_RATE, "STRM_AIRSPEED", 20); // Rate of airspeed stream (Hz) | 0 |  50
  init_param_int(PARAM_STREAM_SONAR_RATE, "STRM_SONAR", 40); // Rate of sonar stream (Hz) | 0 | 40

  init_param_int(PARAM_STREAM_OUTPUT_RAW_RATE, "STRM_SERVO", 50); // Rate of raw output stream | 0 |  490
  init_param_int(PARAM_STREAM_RC_RAW_RATE, "STRM_RC", 50); // Rate of raw RC input stream | 0 | 50

  /********************************/
  /*** CONTROLLER CONFIGURATION ***/
  /********************************/
  init_param_float(PARAM_MAX_COMMAND, "PARAM_MAX_CMD", 1.0); // saturation point for PID controller output | 0 | 1.0

  init_param_float(PARAM_PID_ROLL_RATE_P, "PID_ROLL_RATE_P", 0.070f); // Roll Rate Proportional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_RATE_I, "PID_ROLL_RATE_I", 0.000f); // Roll Rate Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_RATE_D, "PID_ROLL_RATE_D", 0.000f); // Rall Rate Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_PITCH_RATE_P, "PID_PITCH_RATE_P", 0.070f);  // Pitch Rate Proporitional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_RATE_I, "PID_PITCH_RATE_I", 0.0000f); // Pitch Rate Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_RATE_D, "PID_PITCH_RATE_D", 0.0000f); // Pitch Rate Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_YAW_RATE_P, "PID_YAW_RATE_P", 0.25f);   // Yaw Rate Proporitional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_YAW_RATE_I, "PID_YAW_RATE_I", 0.0f);  // Yaw Rate Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_YAW_RATE_D, "PID_YAW_RATE_D", 0.0f);  // Yaw Rate Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_ROLL_ANGLE_P, "PID_ROLL_ANG_P", 0.15f);   // Roll Angle Proporitional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_ANGLE_I, "PID_ROLL_ANG_I", 0.0f);   // Roll Angle Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_ROLL_ANGLE_D, "PID_ROLL_ANG_D", 0.05f);  // Roll Angle Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_PID_PITCH_ANGLE_P, "PID_PITCH_ANG_P", 0.15f);  // Pitch Angle Proporitional Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_ANGLE_I, "PID_PITCH_ANG_I", 0.0f);  // Pitch Angle Integral Gain | 0.0 | 1000.0
  init_param_float(PARAM_PID_PITCH_ANGLE_D, "PID_PITCH_ANG_D", 0.05f); // Pitch Angle Derivative Gain | 0.0 | 1000.0

  init_param_float(PARAM_X_EQ_TORQUE, "X_EQ_TORQUE", 0.0f); // Equilibrium torque added to output of controller on x axis | -1.0 | 1.0
  init_param_float(PARAM_Y_EQ_TORQUE, "Y_EQ_TORQUE", 0.0f); // Equilibrium torque added to output of controller on y axis | -1.0 | 1.0
  init_param_float(PARAM_Z_EQ_TORQUE, "Z_EQ_TORQUE", 0.0f); // Equilibrium torque added to output of controller on z axis | -1.0 | 1.0

  init_param_float(PARAM_PID_TAU, "PID_TAU", 0.05f); // Dirty Derivative time constant - See controller documentation | 0.0 | 1.0


  /*************************/
  /*** PWM CONFIGURATION ***/
  /*************************/
  init_param_int(PARAM_MOTOR_PWM_SEND_RATE, "MOTOR_PWM_UPDATE", 490); // Refresh rate of motor commands to motors - See motor documentation | 0 | 1000
  init_param_float(PARAM_MOTOR_IDLE_THROTTLE, "MOTOR_IDLE_THR", 0.1); // min throttle command sent to motors when armed (Set above 0.1 to spin when armed) | 0.0 | 1.0
  init_param_float(PARAM_FAILSAFE_THROTTLE, "FAILSAFE_THR", 0.3); // Throttle sent to motors in failsafe condition (set just below hover throttle) | 0.0 | 1.0
  init_param_int(PARAM_MOTOR_MIN_PWM, "MOTOR_MIN_PWM", 1000); // PWM value sent to motor ESCs at zero throttle | 1000 | 2000
  init_param_int(PARAM_MOTOR_MAX_PWM, "MOTOR_MAX_PWM", 2000); // PWM value sent to motor ESCs at full throttle | 1000 | 2000
  init_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED, "ARM_SPIN_MOTORS", true); // Enforce MOTOR_IDLE_THR | 0 | 1

  /*******************************/
  /*** ESTIMATOR CONFIGURATION ***/
  /*******************************/
  init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // Time in ms to initialize estimator | 0 | 100000
  init_param_float(PARAM_FILTER_KP, "FILTER_KP", 0.5f); // estimator proportional gain - See estimator documentation | 0 | 10.0
  init_param_float(PARAM_FILTER_KI, "FILTER_KI", 0.05f); // estimator integral gain - See estimator documentation | 0 | 1.0

  init_param_int(PARAM_FILTER_USE_QUAD_INT, "FILTER_QUAD_INT", 1); // Perform a quadratic averaging of LPF gyro data prior to integration (adds ~20 us to estimation loop on F1 processors) | 0 | 1
  init_param_int(PARAM_FILTER_USE_MAT_EXP, "FILTER_MAT_EXP", 1); // 1 - Use matrix exponential to improve gyro integration (adds ~90 us to estimation loop in F1 processors) 0 - use euler integration | 0 | 1
  init_param_int(PARAM_FILTER_USE_ACC, "FILTER_USE_ACC", 1);  // Use accelerometer to correct gyro integration drift (adds ~70 us to estimation loop) | 0 | 1

  init_param_int(PARAM_CALIBRATE_GYRO_ON_ARM, "CAL_GYRO_ARM", false); // True if desired to calibrate gyros on arm | 0 | 1

  init_param_float(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", 0.3f); // Low-pass filter constant - See estimator documentation | 0 | 1.0
  init_param_float(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", 0.5f); // Low-pass filter constant - See estimator documentation | 0 | 1.0

  init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0.0f); // Constant x-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0.0f); // Constant y-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0.0f); // Constant z-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0.0f); // Constant x-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0.0f); // Constant y-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0.0f); // Constant z-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_X_TEMP_COMP,  "ACC_X_TEMP_COMP", 0.0f); // Linear x-axis temperature compensation constant | -2.0 | 2.0
  init_param_float(PARAM_ACC_Y_TEMP_COMP,  "ACC_Y_TEMP_COMP", 0.0f); // Linear y-axis temperature compensation constant | -2.0 | 2.0
  init_param_float(PARAM_ACC_Z_TEMP_COMP,  "ACC_Z_TEMP_COMP", 0.0f); // Linear z-axis temperature compensation constant | -2.0 | 2.0

  init_param_float(PARAM_MAG_A11_COMP,  "MAG_A11_COMP", 1.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A12_COMP,  "MAG_A12_COMP", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A13_COMP,  "MAG_A13_COMP", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A21_COMP,  "MAG_A21_COMP", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A22_COMP,  "MAG_A22_COMP", 1.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A23_COMP,  "MAG_A23_COMP", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A31_COMP,  "MAG_A31_COMP", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A32_COMP,  "MAG_A32_COMP", 0.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_A33_COMP,  "MAG_A33_COMP", 1.0f); // Soft iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_X_BIAS,  "MAG_X_BIAS", 0.0f); // Hard iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_Y_BIAS,  "MAG_Y_BIAS", 0.0f); // Hard iron compensation constant | -999.0 | 999.0
  init_param_float(PARAM_MAG_Z_BIAS,  "MAG_Z_BIAS", 0.0f); // Hard iron compensation constant | -999.0 | 999.0

  init_param_float(PARAM_BARO_BIAS, "BARO_BIAS", 0.0f); // Barometer measurement bias (Pa) | 0 | inf
  init_param_float(PARAM_GROUND_LEVEL, "GROUND_LEVEL", 1387.0f); // Altitude of ground level (m) | -1000 | 10000

  init_param_float(PARAM_DIFF_PRESS_BIAS, "DIFF_PRESS_BIAS", 0.0f); // Differential Pressure Bias (Pa) | -10 | 10

  /************************/
  /*** RC CONFIGURATION ***/
  /************************/
  init_param_int(PARAM_RC_TYPE, "RC_TYPE", 1); // Type of RC input 0 - Parallel PWM (PWM), 1 - Pulse-Position Modulation (PPM) | 0 | 1
  init_param_int(PARAM_RC_X_CHANNEL, "RC_X_CHN", 0); // RC input channel mapped to x-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_Y_CHANNEL, "RC_Y_CHN", 1); // RC input channel mapped to y-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_Z_CHANNEL, "RC_Z_CHN", 3); // RC input channel mapped to z-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_F_CHANNEL, "RC_F_CHN", 2); // RC input channel mapped to F-axis commands [0 - indexed] | 0 | 3
  init_param_int(PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, "RC_ATT_OVRD_CHN", 4); // RC switch mapped to attitude override [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, "RC_THR_OVRD_CHN", 4); // RC switch channel mapped to throttle override [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,  "RC_ATT_CTRL_CHN", -1); // RC switch channel mapped to attitude control type [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_ARM_CHANNEL, "ARM_CHANNEL", -1); // RC switch channel mapped to arming (only if PARAM_ARM_STICKS is false) [0 indexed, -1 to disable] | 4 | 7
  init_param_int(PARAM_RC_NUM_CHANNELS, "RC_NUM_CHN", 6); // number of RC input channels | 1 | 8

  init_param_int(PARAM_RC_SWITCH_5_DIRECTION, "SWITCH_5_DIR", 1); // RC switch 5 toggle direction | -1 | 1
  init_param_int(PARAM_RC_SWITCH_6_DIRECTION, "SWITCH_6_DIR", 1); // RC switch 6 toggle direction | -1 | 1
  init_param_int(PARAM_RC_SWITCH_7_DIRECTION, "SWITCH_7_DIR", 1); // RC switch 7 toggle direction | -1 | 1
  init_param_int(PARAM_RC_SWITCH_8_DIRECTION, "SWITCH_8_DIR", 1); // RC switch 8 toggle direction | -1 | 1

  init_param_float(PARAM_RC_OVERRIDE_DEVIATION, "RC_OVRD_DEV", 0.1); // RC stick deviation from center for overrride | 0.0 | 1.0
  init_param_int(PARAM_OVERRIDE_LAG_TIME, "OVRD_LAG_TIME", 1000); // RC stick deviation lag time before returning control (ms) | 0 | 100000
  init_param_int(PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE, "MIN_THROTTLE", true); // Take minimum throttle between RC and computer at all times | 0 | 1

  init_param_int(PARAM_RC_ATTITUDE_MODE, "RC_ATT_MODE", 1); // Attitude mode for RC sticks (0: rate, 1: angle). Overridden if RC_ATT_CTRL_CHN is set. | 0 | 1
  init_param_float(PARAM_RC_MAX_ROLL, "RC_MAX_ROLL", 0.786f); // Maximum roll angle command sent by full deflection of RC sticks | 0.0 | 3.14159
  init_param_float(PARAM_RC_MAX_PITCH, "RC_MAX_PITCH", 0.786f); // Maximum pitch angle command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  init_param_float(PARAM_RC_MAX_ROLLRATE, "RC_MAX_ROLLRATE", 3.14159f); // Maximum roll rate command sent by full stick deflection of RC sticks | 0.0 | 9.42477796077
  init_param_float(PARAM_RC_MAX_PITCHRATE, "RC_MAX_PITCHRATE", 3.14159f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159
  init_param_float(PARAM_RC_MAX_YAWRATE, "RC_MAX_YAWRATE", 1.507f); // Maximum pitch command sent by full stick deflection of RC sticks | 0.0 | 3.14159

  /***************************/
  /*** FRAME CONFIGURATION ***/
  /***************************/
  init_param_int(PARAM_MIXER, "MIXER", Mixer::INVALID_MIXER); // Which mixer to choose - See Mixer documentation | 0 | 10

  init_param_int(PARAM_FIXED_WING, "FIXED_WING", false); // switches on passthrough commands for fixedwing operation | 0 | 1
  init_param_int(PARAM_ELEVATOR_REVERSE, "ELEVATOR_REV", 0); // reverses elevator servo output | 0 | 1
  init_param_int(PARAM_AILERON_REVERSE, "AIL_REV", 0); // reverses aileron servo output | 0 | 1
  init_param_int(PARAM_RUDDER_REVERSE, "RUDDER_REV", 0); // reverses rudder servo output | 0 | 1

  /********************/
  /*** ARMING SETUP ***/
  /********************/
  init_param_float(PARAM_ARM_THRESHOLD, "ARM_THRESHOLD", 0.15); // RC deviation from max/min in yaw and throttle for arming and disarming check (us) | 0 | 500
}

void Params::add_callback(std::function<void(int)> callback, uint16_t param_id)
{
  callbacks[param_id] = callback;
  callback(param_id);
}

bool Params::read(void)
{
  if (!RF_.board_.memory_read(&params, sizeof(params_t)))
    return false;

  if (params.version != GIT_VERSION_HASH)
    return false;

  if (params.size != sizeof(params_t) || params.magic_be != 0xBE || params.magic_ef != 0xEF)
    return false;

  if (compute_checksum() != params.chk)
    return false;

  return true;
}

bool Params::write(void)
{
  params.version = GIT_VERSION_HASH;
  params.size = sizeof(params_t);
  params.magic_be = 0xBE;
  params.magic_ef = 0xEF;
  params.chk = compute_checksum();

  if (!RF_.board_.memory_write(&params, sizeof(params_t)))
    return false;
  return true;
}

void Params::change_callback(uint16_t id)
{
  // call the callback function
  if(callbacks[id])
    callbacks[id](id);
}

uint16_t Params::lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
  {
    bool match = true;
    for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++)
    {
      // compare each character
      if (name[i] != params.names[id][i])
      {
        match = false;
        break;
      }

      // stop comparing if end of string is reached
      if (params.names[id][i] == '\0')
        break;
    }

    if (match)
      return id;
  }

  return PARAMS_COUNT;
}

bool Params::set_param_int(uint16_t id, int32_t value)
{
  if (id < PARAMS_COUNT && value != params.values[id].ivalue)
  {
    params.values[id].ivalue = value;
    change_callback(id);
    RF_.comm_manager_.send_param_value(id);
    return true;
  }
  return false;
}

bool Params::set_param_float(uint16_t id, float value)
{
  if (id < PARAMS_COUNT && value != params.values[id].fvalue)
  {
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
  return set_param_by_name_int(name, reinterpret_cast<int32_t &>(value));
}
}
