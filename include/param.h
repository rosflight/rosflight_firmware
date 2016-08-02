#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "mavlink.h"

#define PARAMS_NAME_LENGTH MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN

typedef enum
{
  /******************************/
  /*** HARDWARE CONFIGURATION ***/
  /******************************/
  PARAM_BOARD_REVISION,
  PARAM_BAUD_RATE,

  /*****************************/
  /*** MAVLINK CONFIGURATION ***/
  /*****************************/
  PARAM_SYSTEM_ID,
  PARAM_STREAM_HEARTBEAT_RATE,

  PARAM_STREAM_ATTITUDE_RATE,
  PARAM_STREAM_IMU_RATE,
  PARAM_STREAM_MAG_RATE,
  PARAM_STREAM_AIRSPEED_RATE,
  PARAM_STREAM_BARO_RATE,
  PARAM_STREAM_GPS_RATE,
  PARAM_STREAM_SONAR_RATE,

  PARAM_STREAM_SERVO_OUTPUT_RAW_RATE,
  PARAM_STREAM_RC_RAW_RATE,

  /****************************/
  /*** SYSTEM CONFIGURATION ***/
  /****************************/
  PARAM_LOOPTIME,
  PARAM_DIFF_PRESS_UPDATE,
  PARAM_BARO_UPDATE,
  PARAM_SONAR_UPDATE,
  PARAM_MAG_UPDATE,

  /*****************/
  /*** PID GAINS ***/
  /*****************/
  PARAM_MAX_COMMAND,

  PARAM_PID_ROLL_RATE_P,
  PARAM_PID_ROLL_RATE_I,
  PARAM_MAX_ROLL_RATE,

  PARAM_PID_PITCH_RATE_P,
  PARAM_PID_PITCH_RATE_I,
  PARAM_MAX_PITCH_RATE,

  PARAM_PID_YAW_RATE_P,
  PARAM_PID_YAW_RATE_I,
  PARAM_MAX_YAW_RATE,

  PARAM_PID_ROLL_ANGLE_P,
  PARAM_PID_ROLL_ANGLE_I,
  PARAM_PID_ROLL_ANGLE_D,
  PARAM_MAX_ROLL_ANGLE,

  PARAM_PID_PITCH_ANGLE_P,
  PARAM_PID_PITCH_ANGLE_I,
  PARAM_PID_PITCH_ANGLE_D,
  PARAM_MAX_PITCH_ANGLE,

  PARAM_PID_ALT_P,
  PARAM_PID_ALT_I,
  PARAM_PID_ALT_D,

  /*************************/
  /*** PWM CONFIGURATION ***/
  /*************************/
  PARAM_MOTOR_PWM_SEND_RATE,
  PARAM_MOTOR_IDLE_PWM,
  PARAM_SPIN_MOTORS_WHEN_ARMED,

  /*******************************/
  /*** ESTIMATOR CONFIGURATION ***/
  /*******************************/
  PARAM_INIT_TIME,
  PARAM_FILTER_KP,
  PARAM_FILTER_KI,
  PARAM_STREAM_ADJUSTED_GYRO,
  PARAM_GYRO_X_BIAS,
  PARAM_GYRO_Y_BIAS,
  PARAM_GYRO_Z_BIAS,
  PARAM_ACC_X_BIAS,
  PARAM_ACC_Y_BIAS,
  PARAM_ACC_Z_BIAS,
  PARAM_ACC_X_TEMP_COMP,
  PARAM_ACC_Y_TEMP_COMP,
  PARAM_ACC_Z_TEMP_COMP,

  /************************/
  /*** RC CONFIGURATION ***/
  /************************/
  PARAM_RC_TYPE,
  PARAM_RC_X_CHANNEL,
  PARAM_RC_Y_CHANNEL,
  PARAM_RC_Z_CHANNEL,
  PARAM_RC_F_CHANNEL,
  PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL,
  PARAM_RC_THROTTLE_OVERRIDE_CHANNEL,
  PARAM_RC_ATT_CONTROL_TYPE_CHANNEL,
  PARAM_RC_F_CONTROL_TYPE_CHANNEL,

  PARAM_RC_X_CENTER,
  PARAM_RC_Y_CENTER,
  PARAM_RC_Z_CENTER,
  PARAM_RC_F_BOTTOM,
  PARAM_RC_X_RANGE,
  PARAM_RC_Y_RANGE,
  PARAM_RC_Z_RANGE,
  PARAM_RC_F_RANGE,

  PARAM_RC_OVERRIDE_DEVIATION,
  PARAM_OVERRIDE_LAG_TIME,
  PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,

  PARAM_RC_MAX_ROLL_MRAD,
  PARAM_RC_MAX_PITCH_MRAD,
  PARAM_RC_MAX_ROLLRATE_MRAD_S,
  PARAM_RC_MAX_PITCHRATE_MRAD_S,
  PARAM_RC_MAX_YAWRATE_MRAD_S,

  /***************************/
  /*** FRAME CONFIGURATION ***/
  /***************************/
  PARAM_FIXED_WING,
  PARAM_MIXER,
  PARAM_ELEVATOR_REVERSE,
  PARAM_AILERON_REVERSE,
  PARAM_RUDDER_REVERSE,

  /********************/
  /*** ARMING SETUP ***/
  /********************/
  PARAM_ARM_STICKS,
  PARAM_ARM_CHANNEL,
  PARAM_ARM_THRESHOLD,

  // keep track of size of params array
  PARAMS_COUNT
} param_id_t;

typedef enum
{
  PARAM_TYPE_INT32,
  PARAM_TYPE_FLOAT,
  PARAM_TYPE_INVALID
} param_type_t;

// type definitions
typedef struct
{
  uint8_t version;
  uint16_t size;
  uint8_t magic_be;                       // magic number, should be 0xBE

  int32_t values[PARAMS_COUNT];
  char names[PARAMS_COUNT][PARAMS_NAME_LENGTH];
  param_type_t types[PARAMS_COUNT];

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
 * @brief Set all parameters to default values
 */
void set_param_defaults(void);

/**
 * @brief Read parameter values from non-volatile memory
 * @return True if successful, false otherwise
 */
bool read_params(void);

/**
 * @brief Write current parameter values to non-volatile memory
 * @return True if successful, false otherwise
 */
bool write_params(void);

/**
 * @brief Callback for executing actions that need to be taken when a parameter value changes
 * @param id The ID of the parameter that was changed
 */
void param_change_callback(param_id_t id);

/**
 * @brief Gets the id of a parameter from its name
 * @param name The name of the parameter
 * @return The ID of the parameter if the name is valid, PARAMS_COUNT otherwise (invalid ID)
 */
param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH]);

/**
 * @brief Sets the value of a parameter by ID and calls the parameter change callback
 * @param id The ID of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_id(param_id_t id, int32_t value);

/**
 * @brief Sets the value of a parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_name(const char name[PARAMS_NAME_LENGTH], int32_t value);

/**
 * @brief Sets the value of a floating point parameter by ID and calls the parameter callback
 * @param id The ID of the parameter
 * @param value The new value
 * @return  True if a parameter was changed, false otherwise
 */
bool set_param_by_id_float(param_id_t id, float value);

/**
 * @brief Sets the value of a floating point parameter by name and calls the parameter change callback
 * @param name The name of the parameter
 * @param value The new value
 * @return True if a parameter value was changed, false otherwise
 */
bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value);

/**
 * @brief Get the value of a floating point parameter by id
 * @param id The ID of the parameter
 * @return The value of the parameter
 */
float get_param_float(param_id_t id);

#ifdef __cplusplus
}
#endif
