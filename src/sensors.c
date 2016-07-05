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
int16_t _imu_temperature;
uint32_t _imu_time;
bool _imu_ready;
bool calib_gyro;
bool calib_acc;

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

void imu_ISR(void)
{
  _imu_time = micros();
  _imu_ready = true;
}

bool calibrate_acc(void)
{
  calib_acc = true;
  return true;
}

bool calibrate_gyro(void)
{
  calib_gyro = true;
  return true;
}


// local function definitions
static bool update_imu(void)
{
  if (_imu_ready)
  {
    _imu_ready = false;
    mpu6050_read_accel(_accel_data);
    mpu6050_read_gyro(_gyro_data);
    mpu6050_read_temperature(&_imu_temperature);

    if(calib_acc)
    {
      static int16_t acc_count = 0;
      static int32_t acc_sum[3] = {0, 0, 0};
      acc_sum[0] += _accel_data[0];
      acc_sum[1] += _accel_data[1];
      acc_sum[2] += ((_accel_data[2]*_accel_scale)-9807000)/_accel_scale;
      acc_count++;
      if(acc_count > 100)
      {
        _params.values[PARAM_ACC_X_BIAS] = acc_sum[0]/acc_count;
        _params.values[PARAM_ACC_Y_BIAS] = acc_sum[1]/acc_count;
        _params.values[PARAM_ACC_Z_BIAS] = acc_sum[2]/acc_count;
        acc_count = 0;
        acc_sum[0] = 0; acc_sum[1] = 0; acc_sum[2] = 0;
        calib_acc = false;
        // we could do some sanity checking here if we wanted to.
      }

      if(calib_gyro)
      {
        static int16_t gyro_count = 0;
        static int32_t gyro_sum[3] = {0, 0, 0};
        gyro_sum[0] += _gyro_data[0];
        gyro_sum[1] += _gyro_data[1];
        gyro_sum[2] += _gyro_data[2];
        gyro_count++;
        if(gyro_count > 100)
        {
          _params.values[PARAM_GYRO_X_BIAS] = gyro_sum[0]/gyro_count;
          _params.values[PARAM_GYRO_Y_BIAS] = gyro_sum[1]/gyro_count;
          _params.values[PARAM_GYRO_Z_BIAS] = gyro_sum[2]/gyro_count;
          gyro_count = 0;
          gyro_sum[0] = 0; gyro_sum[1] = 0; gyro_sum[2] = 0;
          calib_gyro = false;
          // we could do some sanity checking here if we wanted to.
        }
      }
    }



    // correct according to known biases and temperature compensation
    _accel_data[0] -= (_params.values[PARAM_ACC_X_TEMP_COMP]*_imu_temperature)/1000 + _params.values[PARAM_ACC_X_BIAS];
    _accel_data[1] -= (_params.values[PARAM_ACC_Y_TEMP_COMP]*_imu_temperature)/1000 + _params.values[PARAM_ACC_Y_BIAS];
    _accel_data[2] -= (_params.values[PARAM_ACC_Z_TEMP_COMP]*_imu_temperature)/1000 + _params.values[PARAM_ACC_X_BIAS];
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
  uint16_t acc1G;
  float gyro_scale;
  mpu6050_register_interrupt_cb(&imu_ISR);
  mpu6050_init(true, &acc1G, &gyro_scale, _params.values[PARAM_BOARD_REVISION]);
  _accel_scale = (1000*9807)/acc1G; // convert to um/s^2
  _gyro_scale = (int32_t)(gyro_scale*1000000000.0f); // convert to mrad/s
  imu_last_us = 0;

  // DIFF PRESSURE
  _diff_pressure_present = ms4525_detect();
  diff_press_next_us = 0;
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
    ms5611_update();
    _baro_pressure = ms5611_read_pressure();
    _baro_temperature = ms5611_read_temperature();
  }
  return update_imu();
}

