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
#include "estimator.h"
#include "mode.h"

#include "turbotrig/turbovec.h"

//==================================================================
// global variable definitions

// IMU
vector_t _accel;
vector_t _gyro;
float _imu_temperature;
uint64_t _imu_time;
bool new_imu_data = false;

// Airspeed
bool _diff_pressure_present = false;
float _pitot_velocity, _pitot_diff_pressure, _pitot_temp;

// Barometer
bool _baro_present = false;
float _baro_altitude;
float _baro_pressure;
float _baro_temperature;

// Sonar
bool _sonar_present = false;
float _sonar_range;

// Magnetometer
bool _mag_present = false;
vector_t _mag;


//==================================================================
// local variable definitions

// IMU stuff
int16_t accel_raw[3];
int16_t gyro_raw[3];
int16_t temp_raw;
static volatile uint8_t accel_status, gyro_status, temp_status;
static float accel_scale;
static float gyro_scale;
static bool calibrating_acc_flag;
static bool calibrating_gyro_flag;
static void calibrate_accel(void);
static void calibrate_gyro(void);
static void correct_imu(void);
static void imu_ISR(void);
static bool update_imu(void);


//==================================================================
// function definitions
void init_sensors(void)
{
  while(millis() < 50);
  i2cWrite(0,0,0);
  _baro_present = ms5611_init();
  _mag_present = hmc5883lInit(get_param_int(PARAM_BOARD_REVISION));
  _sonar_present = mb1242_init();
  _diff_pressure_present = ms4525_init();

  // IMU
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &gyro_scale, get_param_int(PARAM_BOARD_REVISION));
  mpu6050_register_interrupt_cb(&imu_ISR);
  accel_scale = 9.80665f/acc1G * get_param_float(PARAM_ACCEL_SCALE);
}


bool update_sensors()
{
  // Look for disabled sensors while disarmed (poll every 10 seconds)
  // These sensors need power to respond, so they might not have been
  // detected on startup, but will be detected whenever power is applied
  // to the 5V rail.
  static uint32_t last_time_look_for_disarmed_sensors = 0;
  if ((_armed_state == DISARMED && (!_sonar_present )) || (!_diff_pressure_present))
  {
    uint32_t now = millis();
    if (now > (last_time_look_for_disarmed_sensors + 500))
    {
      last_time_look_for_disarmed_sensors = now;
      if(!_sonar_present)
      {
        mb1242_update();
        _sonar_present = (mb1242_read() > 0.2);
        if(_sonar_present)
        {
          mavlink_log_info("FOUND SONAR", NULL);
        }
      }
      if(!_diff_pressure_present)
      {
        _diff_pressure_present = ms4525_init();
        if(_diff_pressure_present)
        {
          mavlink_log_info("FOUND DIFF PRESS", NULL);
        }
      }
      if(!_mag_present)
      {
        _mag_present = hmc5883lInit(get_param_int(PARAM_BOARD_REVISION));
        if(_mag_present)
        {
          mavlink_log_info("FOUND MAG", NULL);
        }
      }
    }
  }

  if(_baro_present)
  {
    ms5611_update();
    ms5611_read(&_baro_altitude, &_baro_pressure, &_baro_temperature);
  }

    if(_diff_pressure_present)
    {

      ms4525_update();
      ms4525_read(&_pitot_diff_pressure, &_pitot_temp, &_pitot_velocity);
    }

  if (_sonar_present)
  {
    mb1242_update();
    _sonar_range = mb1242_read();
  }

  if (_mag_present)
  {
    int16_t raw_mag[3] = {0,0,0};
    hmc5883l_update();
    hmc5883l_read(raw_mag);
    _mag.x = (float)raw_mag[0];
    _mag.y = (float)raw_mag[1];
    _mag.z = (float)raw_mag[2];
  }

  // Return whether or not we got new IMU data
  return update_imu();
}


bool start_imu_calibration(void)
{
  start_gyro_calibration();

  calibrating_acc_flag = true;
  set_param_float(PARAM_ACC_X_BIAS, 0.0);
  set_param_float(PARAM_ACC_Y_BIAS, 0.0);
  set_param_float(PARAM_ACC_Z_BIAS, 0.0);
  return true;
}

bool start_gyro_calibration(void)
{
  calibrating_gyro_flag = true;
  set_param_float(PARAM_GYRO_X_BIAS, 0.0);
  set_param_float(PARAM_GYRO_Y_BIAS, 0.0);
  set_param_float(PARAM_GYRO_Z_BIAS, 0.0);
  return true;
}

bool gyro_calibration_complete(void)
{
  return !calibrating_gyro_flag;
}




//==================================================================
// local function definitions
void imu_ISR(void)
{
  _imu_time = micros();
  new_imu_data = true;
}


static bool update_imu(void)
{
  static uint32_t last_imu_update_ms = 0;

  if(new_imu_data)
  {
    last_imu_update_ms = millis();
    mpu6050_read_accel(accel_raw);
    mpu6050_read_gyro(gyro_raw);
    mpu6050_read_temperature(&temp_raw);
    new_imu_data = false;

    // convert temperature SI units (degC, m/s^2, rad/s)
    _imu_temperature = temp_raw/340.0f + 36.53f;

    // convert to NED and SI units
    _accel.x = accel_raw[0] * accel_scale;
    _accel.y = -accel_raw[1] * accel_scale;
    _accel.z = -accel_raw[2] * accel_scale;

    _gyro.x = gyro_raw[0] * gyro_scale;
    _gyro.y = -gyro_raw[1] * gyro_scale;
    _gyro.z = -gyro_raw[2] * gyro_scale;

    if (calibrating_acc_flag == true)
      calibrate_accel();
    if (calibrating_gyro_flag)
      calibrate_gyro();

    correct_imu();
    return true;
  }
  else
  {
    // if we have lost 1000 IMU messages something is wrong
    if(millis() > last_imu_update_ms + 1000)
    {
      // change board revision and reset IMU
      last_imu_update_ms = millis();
      set_param_int(PARAM_BOARD_REVISION, (get_param_int(PARAM_BOARD_REVISION) >= 4) ? 2 : 5);
      uint16_t acc1G;
      mpu6050_init(true, &acc1G, &gyro_scale, get_param_int(PARAM_BOARD_REVISION));
      accel_scale = 9.80665f/acc1G * get_param_float(PARAM_ACCEL_SCALE);
    }
    return false;
  }
}


static void calibrate_gyro()
{
  static uint16_t count = 0;
  static vector_t gyro_sum  = { 0.0f, 0.0f, 0.0f };
  gyro_sum = vector_add(gyro_sum, _gyro);
  count++;

  if(count > 100)
  {
    // Gyros are simple.  Just find the average during the calibration
    vector_t gyro_bias = scalar_multiply(1.0/(float)count, gyro_sum);

    if(sqrd_norm(gyro_bias) < 1.0)
    {
      set_param_float(PARAM_GYRO_X_BIAS, gyro_bias.x);
      set_param_float(PARAM_GYRO_Y_BIAS, gyro_bias.y);
      set_param_float(PARAM_GYRO_Z_BIAS, gyro_bias.z);

      // Tell the estimator to reset it's bias estimate, because it should be zero now
      reset_adaptive_bias();
    }
    else
    {
      mavlink_log_error("Too much movement for gyro cal", NULL);
    }

    // reset calibration in case we do it again
    calibrating_gyro_flag = false;
    count = 0;
    gyro_sum.x = 0.0f;
    gyro_sum.y = 0.0f;
    gyro_sum.z = 0.0f;
  }
}


static void calibrate_accel(void)
{
  static uint16_t count = 0;
  static vector_t acc_sum  = { 0.0f, 0.0f, 0.0f };
  static const vector_t gravity = {0.0f, 0.0f, 9.80665f};
  static float acc_temp_sum = 0.0f;

  acc_sum = vector_add(vector_add(acc_sum, _accel), gravity);
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

    // Sanity Check -
    // If the accelerometer is upside down or being spun around during the calibration,
    // then don't do anything
    if(sqrd_norm(accel_bias) < 4.5)
    {
      set_param_float(PARAM_ACC_X_BIAS, accel_bias.x);
      set_param_float(PARAM_ACC_Y_BIAS, accel_bias.y);
      set_param_float(PARAM_ACC_Z_BIAS, accel_bias.z);
      mavlink_log_info("IMU offsets captured", NULL);

      // reset the estimated state
      reset_state();
      calibrating_acc_flag = false;
    }
    else
    {
      // check for bad _accel_scale
      if(sqrd_norm(accel_bias) > 4.5*4.5 && sqrd_norm(accel_bias) < 5.5*5.5)
      {
        mavlink_log_error("Detected bad IMU accel scale value", 0);
        set_param_float(PARAM_ACCEL_SCALE, 2.0 * get_param_float(PARAM_ACCEL_SCALE));
        accel_scale *= get_param_float(PARAM_ACCEL_SCALE);
        write_params();
      }
      else if (sqrd_norm(accel_bias) > 9.0*9.0 && sqrd_norm(accel_bias) < 11.0*11.0)
      {
        mavlink_log_error("Detected bad IMU accel scale value", 0);
        set_param_float(PARAM_ACCEL_SCALE, 0.5 * get_param_float(PARAM_ACCEL_SCALE));
        accel_scale *= get_param_float(PARAM_ACCEL_SCALE);
        write_params();
      }
      else
      {
        mavlink_log_error("Too much movement for IMU cal", NULL);
        calibrating_acc_flag = false;
      }
    }

    // reset calibration in case we do it again
    count = 0;
    acc_sum.x = 0.0f;
    acc_sum.y = 0.0f;
    acc_sum.z = 0.0f;
    acc_temp_sum = 0.0f;
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
