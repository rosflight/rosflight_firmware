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

// global variable definitions
bool _imu_ready;
vector_t _accel;
vector_t _gyro;
float _imu_temperature;
uint32_t _imu_time;

bool _diff_pressure_present;
int16_t _diff_pressure;
int16_t _temperature;

bool _baro_present;
int16_t _baro_pressure;
int16_t _baro_temperature;

bool _sonar_present;
int16_t _sonar_range;
uint32_t _sonar_time;

// local variable definitions
static uint32_t diff_press_next_us;
static uint32_t baro_next_us;
static uint32_t sonar_next_us;

static float accel_scale;
static float gyro_scale;

static bool calib_gyro;
static bool calib_acc;

// local function definitions
void imu_ISR(void)
{
  _imu_time = micros();
  _imu_ready = true;
}

static bool update_imu(void)
{
  if (_imu_ready)
  {
    _imu_ready = false;

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t imu_temp_raw;

    mpu6050_read_accel(accel_raw);
    mpu6050_read_gyro(gyro_raw);
    mpu6050_read_temperature(&imu_temp_raw);

    // convert temperature SI units (degC, m/s^2, rad/s)
    _imu_temperature = imu_temp_raw/340.0f + 36.53f;

    _accel.x = accel_raw[0] * accel_scale;
    _accel.y = accel_raw[1] * accel_scale;
    _accel.z = accel_raw[2] * accel_scale;

    _gyro.x = gyro_raw[0] * gyro_scale;
    _gyro.y = gyro_raw[1] * gyro_scale;
    _gyro.z = gyro_raw[2] * gyro_scale;

    if (calib_acc)
    {
      static uint16_t acc_count = 0;
      static vector_t acc_sum  = { 0.0f, 0.0f, 0.0f };
      static float acc_temp_sum = 0.0f;

      acc_sum.x += _accel.x;
      acc_sum.y += _accel.y;
      acc_sum.z += _accel.z - 9.80665f;
      acc_temp_sum += _imu_temperature;
      acc_count++;

      if (acc_count > 1000)
      {
        set_param_by_id_float(PARAM_ACC_X_BIAS,
                              (acc_sum.x - get_param_float(PARAM_ACC_X_TEMP_COMP) * acc_temp_sum) / (float)acc_count);
        set_param_by_id_float(PARAM_ACC_Y_BIAS,
                              (acc_sum.y - get_param_float(PARAM_ACC_Y_TEMP_COMP) * acc_temp_sum) / (float)acc_count);
        set_param_by_id_float(PARAM_ACC_Z_BIAS,
                              (acc_sum.z - get_param_float(PARAM_ACC_Z_TEMP_COMP) * acc_temp_sum) / (float)acc_count);

        acc_count = 0;
        acc_sum.x = 0.0f;
        acc_sum.y = 0.0f;
        acc_sum.z = 0.0f;
        acc_temp_sum = 0.0f;
        calib_acc = false;
        // we could do some sanity checking here if we wanted to.
      }
    }

    if (calib_gyro)
    {
      static uint16_t gyro_count = 0;
      static vector_t gyro_sum = { 0.0f, 0.0f, 0.0f };

      gyro_sum.x += _gyro.x;
      gyro_sum.y += _gyro.y;
      gyro_sum.z += _gyro.z;
      gyro_count++;

      if (gyro_count > 1000)
      {
        set_param_by_id_float(PARAM_GYRO_X_BIAS, gyro_sum.x / (float)gyro_count);
        set_param_by_id_float(PARAM_GYRO_Y_BIAS, gyro_sum.y / (float)gyro_count);
        set_param_by_id_float(PARAM_GYRO_Z_BIAS, gyro_sum.z / (float)gyro_count);

        gyro_count = 0;
        gyro_sum.x = 0.0f;
        gyro_sum.y = 0.0f;
        gyro_sum.z = 0.0f;
        calib_gyro = false;
        // we could do some sanity checking here if we wanted to.
      }
    }

    // correct according to known biases and temperature compensation
    _accel.x -= get_param_float(PARAM_ACC_X_TEMP_COMP)*_imu_temperature + get_param_float(PARAM_ACC_X_BIAS);
    _accel.y -= get_param_float(PARAM_ACC_Y_TEMP_COMP)*_imu_temperature + get_param_float(PARAM_ACC_Y_BIAS);
    _accel.z -= get_param_float(PARAM_ACC_Z_TEMP_COMP)*_imu_temperature + get_param_float(PARAM_ACC_Z_BIAS);

    _gyro.x -= get_param_float(PARAM_GYRO_X_BIAS);
    _gyro.y -= get_param_float(PARAM_GYRO_Y_BIAS);
    _gyro.z -= get_param_float(PARAM_GYRO_Z_BIAS);
    return true;
  }
  else
  {
    return false;
  }
}

// function definitions
void init_sensors(void)
{
  // BAROMETER <-- for some reason, this has to come first
  i2cWrite(0,0,0);
  _baro_present = ms5611_init();
  baro_next_us = 0;

  // IMU
  _imu_ready = false;
  mpu6050_register_interrupt_cb(&imu_ISR);
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &gyro_scale, _params.values[PARAM_BOARD_REVISION]);
  accel_scale = 9.80665f/acc1G;
  calib_gyro = true;
  calib_acc = true;

  // DIFF PRESSURE
  _diff_pressure_present = ms4525_detect();
  diff_press_next_us = 0;

  // SONAR
  _sonar_present = mb1242_init();
  sonar_next_us = 0;
}

bool update_sensors(uint32_t time_us)
{
  // using else so that we don't do all sensor updates on the same loop
  if (_diff_pressure_present && time_us >= diff_press_next_us)
  {
    diff_press_next_us += _params.values[PARAM_DIFF_PRESS_UPDATE];
    ms4525_read(&_diff_pressure, &_temperature);
  }
  else if (_baro_present && time_us > baro_next_us)
  {
    baro_next_us += _params.values[PARAM_BARO_UPDATE];
    _baro_pressure = ms5611_read_pressure();
    _baro_temperature = ms5611_read_temperature();
  }
  else if (_sonar_present && time_us > sonar_next_us)
  {
    sonar_next_us += _params.values[PARAM_SONAR_UPDATE];
    _sonar_time = micros();
    _sonar_range = mb1242_poll();
  }
  return update_imu();
}

bool calibrate_acc(void)
{
  calib_acc = true;
  set_param_by_id_float(PARAM_ACC_X_BIAS, 0.0);
  set_param_by_id_float(PARAM_ACC_Y_BIAS, 0.0);
  set_param_by_id_float(PARAM_ACC_Z_BIAS, 0.0);
  return true;
}

bool calibrate_gyro(void)
{
  calib_gyro = true;
  set_param_by_id_float(PARAM_GYRO_X_BIAS, 0.0);
  set_param_by_id_float(PARAM_GYRO_Y_BIAS, 0.0);
  set_param_by_id_float(PARAM_GYRO_Z_BIAS, 0.0);
  return true;
}

#ifdef __cplusplus
}
#endif
