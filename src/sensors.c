#include <stdbool.h>
#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "param.h"
#include "sensors.h"

// global variable definitions
int16_t _accel_data[3];
int16_t _gyro_data[3];
int32_t _accel_scale;
int32_t _gyro_scale;

// local variable definitions
static uint32_t imu_last_us;


static void correct_imu(void)
{
  _accel_data[0] += _params.values[PARAM_ACC_X_BIAS];
  _accel_data[1] += _params.values[PARAM_ACC_Y_BIAS];
  _accel_data[2] += _params.values[PARAM_ACC_Z_BIAS];
  _gyro_data[0] += _params.values[PARAM_GYRO_X_BIAS];
  _gyro_data[1] += _params.values[PARAM_GYRO_Y_BIAS];
  _gyro_data[2] += _params.values[PARAM_GYRO_Z_BIAS];
}

// local function definitions
static bool update_imu(void)
{
  if (mpuDataReady)
  {
    mpu6050_read_accel(_accel_data);
    mpu6050_read_gyro(_gyro_data);
    correct_imu();
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
  mpu6050_init(true, &acc1G, &gyro_scale);
  _accel_scale = (1000*9807)/acc1G; // convert to um/s^2
  _gyro_scale = (int32_t)(gyro_scale*1000000000.0f); // convert to mrad/s
  imu_last_us = 0;
}

bool update_sensors(uint32_t time_us)
{
  if (update_imu())
  {
    return true;
  }
  else
    return false;
}

