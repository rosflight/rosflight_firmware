#include <stdbool.h>
#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mavlink_util.h"
#include "param.h"
#include "sensors.h"

// global variable definitions
int16_t _accel_data[3];
int16_t _gyro_data[3];
int32_t _accel_scale;
int32_t _gyro_scale;

bool _diff_pressure_present;
int16_t _diff_pressure;
int16_t _temperature;

bool _baro_present;
int16_t _baro_pressure;
int16_t _baro_temperature;

bool _sonar_present;
int16_t _sonar_range;


// local variable definitions
static uint32_t imu_last_us;
static uint32_t diff_press_next_us;
static uint32_t baro_next_us;


// local function definitions
static bool update_imu(void)
{
  if(mpuDataReady)
  {
    mpu6050_read_accel(_accel_data);
    mpu6050_read_gyro(_gyro_data);
    // correct according to known biases
    _accel_data[0] += _params.values[PARAM_ACC_X_BIAS];
    _accel_data[1] += _params.values[PARAM_ACC_Y_BIAS];
    _accel_data[2] += _params.values[PARAM_ACC_Z_BIAS];
    _gyro_data[0] += _params.values[PARAM_GYRO_X_BIAS];
    _gyro_data[1] += _params.values[PARAM_GYRO_Y_BIAS];
    _gyro_data[2] += _params.values[PARAM_GYRO_Z_BIAS];
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
  // IMU
  uint16_t acc1G;
  float gyro_scale;
  mpu6050_init(true, &acc1G, &gyro_scale, _params.values[PARAM_BOARD_REVISION]);
  _accel_scale = (1000*9807)/acc1G; // convert to um/s^2
  _gyro_scale = (int32_t)(gyro_scale*1000000000.0f); // convert to mrad/s
  imu_last_us = 0;

  // BAROMETER <-- for some reason, this has to come before diff pressure
  _baro_present = ms5611_init();
  baro_next_us = 0;

  // DIFF PRESSURE
  _diff_pressure_present = ms4525_detect();
  diff_press_next_us = 0;
}

bool update_sensors(uint32_t time_us)
{
  // using else so that we don't do all sensor updates on the same loop
  if (_diff_pressure_present && time_us >= diff_press_next_us )
  {
    diff_press_next_us += _params.values[PARAM_DIFF_PRESS_UPDATE];
    ms4525_read(&_diff_pressure, &_temperature);
  }
  else if (_baro_present && time_us > baro_next_us)
  {
    baro_next_us += _params.values[PARAM_BARO_UPDATE];
    ms5611_update();
    _baro_pressure = ms5611_read_pressure();
    _baro_temperature = ms5611_read_temperature();
  }
  return update_imu();
}

