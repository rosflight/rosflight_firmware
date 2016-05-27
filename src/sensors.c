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

// local function definitions
static void update_imu(void)
{
  mpu6050_read_accel(_accel_data);
  mpu6050_read_gyro(_gyro_data);
}

// function definitions
void init_sensors(void)
{
  // IMU
  uint16_t acc1G;
  float gyro_scale;
  mpu6050_init(true, &acc1G, &gyro_scale);
  _accel_scale = 9807 / acc1G; // convert to mm/s^2
  _gyro_scale = (int32_t) (1e6 * gyro_scale); // convert to urad/s
  imu_last_us = 0;
}

bool update_sensors(uint32_t time_us)
{
  if (time_us - imu_last_us >= 5000)
  {
    imu_last_us = time_us;
    update_imu();
    return true;
  }
  else
    return false;
}
