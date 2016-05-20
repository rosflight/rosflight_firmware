#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "mavlink.h"

#define PARAMS_COUNT 60
#define PARAMS_NAME_LENGTH MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN

/*********************************/
/*** 0-10 SERIAL CONFIGURATION ***/
/*********************************/

#define PARAM_SYSTEM_ID 0
#define PARAM_STREAM_HEARTBEAT_RATE 1
#define PARAM_STREAM_IMU_RATE 2
#define PARAM_STREAM_MAG_RATE 3
#define PARAM_STREAM_AIRSPEED_RATE 4
#define PARAM_STREAM_GPS_RATE 5
#define PARAM_STREAM_SONAR_RATE 6
#define PARAM_LOOPTIME 7

/***********************/
/*** 11-30 PID GAINS ***/
/***********************/

#define PARAM_PID_ROLL_RATE_P 11
#define PARAM_PID_ROLL_RATE_I 12
#define PARAM_PID_ROLL_RATE_D 13

#define PARAM_PID_PITCH_RATE_P 14
#define PARAM_PID_PITCH_RATE_I 15
#define PARAM_PID_PITCH_RATE_D 16

#define PARAM_PID_YAW_RATE_P 17
#define PARAM_PID_YAW_RATE_I 18
#define PARAM_PID_YAW_RATE_D 19

#define PARAM_PID_ROLL_ANGLE_P 20
#define PARAM_PID_ROLL_ANGLE_I 21
#define PARAM_PID_ROLL_ANGLE_D 22

#define PARAM_PID_PITCH_ANGLE_P 23
#define PARAM_PID_PITCH_ANGLE_I 24
#define PARAM_PID_PITCH_ANGLE_D 25

#define PARAM_PID_ALT_P 26
#define PARAM_PID_ALT_I 27
#define PARAM_PID_ALT_D 28

/*******************************/
/*** 31-40 PWM CONFIGURATION ***/
/*******************************/

#define PARAM_MOTOR_PWM_SEND_RATE 31

#define PARAM_RC_ROLL_CHANNEL 32
#define PARAM_RC_PITCH_CHANNEL 33
#define PARAM_RC_YAW_CHANNEL 34
#define PARAM_RC_THROTTLE_CHANNEL 35
#define PARAM_RC_TYPE 36 // 0 is PWM, 1 is PPM
#define PARAM_IDLE_PWM 37

/*************************************/
/*** 41-50 ESTIMATOR CONFIGURATION ***/
/*************************************/

#define PARAM_ESTIMATOR_LPF_ALPHA 41
#define PARAM_GYRO_LPF_ALPHA 42
#define PARAM_ACC_LPF_ALPHA 43

// type definitions
typedef struct
{
  uint8_t version;
  uint16_t size;
  uint8_t magic_be;                       // magic number, should be 0xBE

  int32_t values[PARAMS_COUNT];
  char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];

  uint8_t magic_ef;                       // magic number, should be 0xEF
  uint8_t chk;                            // XOR checksum
} params_t;

// global variable declarations
extern params_t _params;

// function declarations

/**
 * @brief Initialize parameter values
 */
void init_params(void);

/**
 * @brief Callback for executing actions that need to be taken when a parameter value changes
 * @param id The ID of the parameter that was changed
 */
void param_change_callback(uint8_t id);

/**
 * @brief Gets the id of a parameter from its name
 * @param name The name of the parameter
 * @returns The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
 */
uint8_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

/**
 * @brief Sets the value of a parameter by ID and calls the parameter change callback
 * @param id The ID of the parameter
 * @param value The new value
 * @returns True if a parameter value was changed, false otherwise
 */
inline bool set_param_by_id(uint8_t id, int32_t value)
{
  if (id < PARAMS_COUNT && value != _params.values[id])
  {
    _params.values[id] = value;
    param_change_callback(id);
    return true;
  }
  return false;
}

/**
 * @brief Sets the value of a parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @returns True if a parameter value was changed, false otherwise
 */
inline bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  uint8_t id = lookup_param_id(name);
  return set_param_by_id(id, value);
}
