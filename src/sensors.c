#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_util.h"
#include "mavlink_log.h"
#include "param.h"
#include "sensors.h"

#include "turbotrig/turbovec.h"

//==================================================================
// global variable definitions

// IMU
vector_t _accel;
vector_t _gyro;
float _imu_temperature;
uint32_t _imu_time;

// Airspeed
bool _diff_pressure_present = false;
int16_t _diff_pressure;
int16_t _diff_pressure_temperature;

// Barometer
bool _baro_present = false;
int32_t _baro_pressure;
int32_t _baro_temperature;

// Sonar
bool _sonar_present = false;
int16_t _sonar_range;
uint32_t _sonar_time;

// Magnetometer
bool _mag_present = false;
vector_t _mag;
uint32_t _mag_time;


//==================================================================
// local variable definitions

// IMU stuff
int16_t accel_raw[3];
int16_t gyro_raw[3];
int16_t temp_raw;
static volatile uint8_t accel_status, gyro_status, temp_status;
static float accel_scale;
static float gyro_scale;
static bool calibrating_imu_flag;
static void calibrate_imu(void);
static void correct_imu(void);
static void imu_ISR(void);
static bool update_imu(void);


//==================================================================
// function definitions
void init_sensors(void)
{
  // BAROMETER <-- for some reason, this has to come first
  i2cWrite(0,0,0);
  _baro_present = ms5611_init();

  // MAGNETOMETER
  _mag_present = hmc5883lInit(2);

  // SONAR -- Not working yet
  _sonar_present = mb1242_init();

  // DIFF PRESSURE
//  _diff_pressure_present = ms4525_detect(); ///<-This breaks the IMU.  I don't know why
//  diff_press_next_us = 0;

  // IMU
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &gyro_scale, _params.values[PARAM_BOARD_REVISION]);
  mpu6050_register_interrupt_cb(&imu_ISR);
  accel_scale = 9.80665f/acc1G;
}


bool update_sensors()
{
  if(_baro_present)
  {
    ms5611_request_async_update();
    _baro_pressure = ms5611_read_pressure();
    _baro_temperature = ms5611_read_temperature();
  }

  if(_diff_pressure_present)
  {
//    ms4525_request_async_update();
//    _diff_pressure = ms4525_read_velocity();
//    _diff_pressure_temperature = ms4525_read_temperature();
  }

  if (_sonar_present)
  {
    _sonar_time = micros();
    _sonar_range = mb1242_poll();
  }

  if (_mag_present)
  {
    int16_t raw_mag[3] = {0,0,0};
    hmc5883l_request_async_update();
    hmc5883l_read_magnetometer(raw_mag);
    _mag.x = (float)raw_mag[0];
    _mag.y = (float)raw_mag[1];
    _mag.z = (float)raw_mag[2];
    _mag = vector_normalize(_mag);
  }

  // Return whether or not we got new IMU data
  return update_imu();
}


bool start_imu_calibration(void)
{
  calibrating_imu_flag = true;
  set_param_by_id_float(PARAM_ACC_X_BIAS, 0.0);
  set_param_by_id_float(PARAM_ACC_Y_BIAS, 0.0);
  set_param_by_id_float(PARAM_ACC_Z_BIAS, 0.0);

  set_param_by_id_float(PARAM_GYRO_X_BIAS, 0.0);
  set_param_by_id_float(PARAM_GYRO_Y_BIAS, 0.0);
  set_param_by_id_float(PARAM_GYRO_Z_BIAS, 0.0);
  return true;
}




//==================================================================
// local function definitions
void imu_ISR(void)
{
  _imu_time = micros();

  mpu6050_request_async_accel_read(accel_raw, &accel_status);
  mpu6050_request_async_gyro_read(gyro_raw, &gyro_status);
  mpu6050_request_async_temp_read(&temp_raw, &temp_status);
}


static bool update_imu(void)
{
  if (accel_status == I2C_JOB_COMPLETE
      && gyro_status == I2C_JOB_COMPLETE
      && temp_status == I2C_JOB_COMPLETE)
  {
    // reset flags
    accel_status = I2C_JOB_DEFAULT;
    gyro_status = I2C_JOB_DEFAULT;
    temp_status = I2C_JOB_DEFAULT;

    // convert temperature SI units (degC, m/s^2, rad/s)
    _imu_temperature = temp_raw/340.0f + 36.53f;

    // convert to NED and SI units
    _accel.x = accel_raw[0] * accel_scale;
    _accel.y = -accel_raw[1] * accel_scale;
    _accel.z = -accel_raw[2] * accel_scale;

    _gyro.x = gyro_raw[0] * gyro_scale;
    _gyro.y = -gyro_raw[1] * gyro_scale;
    _gyro.z = -gyro_raw[2] * gyro_scale;

    if (calibrating_imu_flag == true)
      calibrate_imu();

    correct_imu();
    return true;
  }
  else
  {
    return false;
  }
}


static void calibrate_imu(void)
{
  static uint16_t count = 0;
  static vector_t acc_sum  = { 0.0f, 0.0f, 0.0f };
  static vector_t gyro_sum  = { 0.0f, 0.0f, 0.0f };
  static const vector_t gravity = {0.0f, 0.0f, 9.80665f};
  static float acc_temp_sum = 0.0f;

  acc_sum = vector_add(vector_add(acc_sum, _accel), gravity);
  gyro_sum = vector_add(gyro_sum, _gyro);
  acc_temp_sum += _imu_temperature;
  count++;

  if (count > 1000)
  {
    // The temperature bias is calculated using a least-squares regression.
    // This is computationally intensive, so it is done by the onboard computer in
    // fcu_io and shipped over to the flight controller.
    vector_t accel_temp_bias = {
      get_param_float(PARAM_ACC_X_TEMP_COMP),
      get_param_float(PARAM_ACC_Y_TEMP_COMP),
      get_param_float(PARAM_ACC_Z_TEMP_COMP)
    };

    // Figure out the proper accel bias.
    // We have to consider the contribution of temperature during the calibration,
    // Which is why this line is so confusing. What we are doing, is first removing
    // the contribution of temperature to the measurements during the calibration,
    // Then we are dividing by the number of measurements.
    vector_t accel_bias = scalar_multiply(1.0/(float)count, vector_sub(acc_sum, scalar_multiply(acc_temp_sum, accel_temp_bias)));

    // Gyros are simple.  Just find the average during the calibration
    vector_t gyro_bias = scalar_multiply(1.0/(float)count, gyro_sum);

    // Sanity Check -
    // If the accelerometer is upside down or being spun around during the calibration,
    // then don't do anything
    if(sqrd_norm(accel_bias) < 1.5*1.5 && sqrd_norm(gyro_bias) < 0.1*0.1)
    {
      set_param_by_id_float(PARAM_ACC_X_BIAS, accel_bias.x);
      set_param_by_id_float(PARAM_ACC_Y_BIAS, accel_bias.y);
      set_param_by_id_float(PARAM_ACC_Z_BIAS, accel_bias.z);

      set_param_by_id_float(PARAM_GYRO_X_BIAS, gyro_bias.x);
      set_param_by_id_float(PARAM_GYRO_Y_BIAS, gyro_bias.y);
      set_param_by_id_float(PARAM_GYRO_Z_BIAS, gyro_bias.z);
    }

    // reset calibration in case we do it again
    count = 0;
    acc_sum.x = 0.0f;
    acc_sum.y = 0.0f;
    acc_sum.z = 0.0f;
    gyro_sum.x = 0.0f;
    gyro_sum.y = 0.0f;
    gyro_sum.z = 0.0f;
    acc_temp_sum = 0.0f;
    calibrating_imu_flag = false;
  }
}


static void correct_imu(void)
{
  // correct according to known biases and temperature compensation
  _accel.x -= get_param_float(PARAM_ACC_X_TEMP_COMP)*_imu_temperature + get_param_float(PARAM_ACC_X_BIAS);
  _accel.y -= get_param_float(PARAM_ACC_Y_TEMP_COMP)*_imu_temperature + get_param_float(PARAM_ACC_Y_BIAS);
  _accel.z -= get_param_float(PARAM_ACC_Z_TEMP_COMP)*_imu_temperature + get_param_float(PARAM_ACC_Z_BIAS);

  _gyro.x -= get_param_float(PARAM_GYRO_X_BIAS);
  _gyro.y -= get_param_float(PARAM_GYRO_Y_BIAS);
  _gyro.z -= get_param_float(PARAM_GYRO_Z_BIAS);
}


#ifdef __cplusplus
}
#endif
