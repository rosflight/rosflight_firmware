#include <stdbool.h>
#include <stdint.h>

#include <drv_mpu6050.h>

#include "param.h"

#include "sensors.h"

// global variable definitions
imuData_t _imu_data;

// local variable definitions
static uint32_t imu_last_us;
static float accel_scale;
static float gyro_scale;

// local function definitions
static void update_imu(void)
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];

  mpu6050_read_accel(accel_raw);
  mpu6050_read_gyro(gyro_raw);

  _imu_data.ax = accel_raw[0] * accel_scale;
  _imu_data.ay = accel_raw[1] * accel_scale;
  _imu_data.az = accel_raw[2] * accel_scale;

  _imu_data.gx = gyro_raw[0] * gyro_scale;
  _imu_data.gy = gyro_raw[1] * gyro_scale;
  _imu_data.gz = gyro_raw[2] * gyro_scale;
}

// function definitions
void init_sensors(void)
{
  // IMU
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &gyro_scale);
  accel_scale = 9.80665f / acc1G;
}

void update_sensors(uint32_t time_us)
{
  if (time_us - imu_last_us >= _params.sensors.imu_period_us)
  {
    update_imu();
    imu_last_us = time_us;
  }
}
