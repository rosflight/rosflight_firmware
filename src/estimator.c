#include <stdlib.h>

#include <breezystm32/breezystm32.h>
#include <turbotrig/turbotrig.h>

#include "sensors.h"
#include "param.h"

#include "estimator.h"

state_t _current_state;

void init_estimator()
{
  _current_state.p = 0;
  _current_state.q = 0;
  _current_state.r = 0;
  _current_state.phi = 0;
  _current_state.theta = 0;
  _current_state.psi = 0;
}


void run_estimator(int32_t now)
{
  static int32_t last_time = 0;
  int32_t dt = now - last_time;
  last_time = now;

  int32_t tau = 100; // desired time constant of the filter (us) <-- should be a param
  int32_t alpha = 1000;  // only trust gyro unless we figure out it's safe to use accel

  // check if z-acceleration is greater than 1.15G and less than 0.85G
  int32_t acc_phi = 0;
  int32_t acc_theta = 0;
  int32_t acc_mag_squared = (_accel_data[2]*_accel_data[2])/1000
                          + (_accel_data[1]*_accel_data[1])/1000
                          + (_accel_data[0]*_accel_data[0])/1000;
  if ( acc_mag_squared < 19294 && acc_mag_squared > 14261)
  {
    // pull in accelerometer data
    acc_phi = turboatan2(_accel_data[1], _accel_data[2]);
    acc_theta = turboatan2(_accel_data[0], _accel_data[2]);

    // calculate filter constant
    alpha = (1000000*tau)/(tau*1000+dt);
  }

  // pull in gyro data
  int32_t meas_p = ((int32_t)_gyro_data[0]*_gyro_scale); // mrad/s
  int32_t meas_q = -1*((int32_t)_gyro_data[1]*_gyro_scale); // Convert to NED
  int32_t meas_r = -1*((int32_t)_gyro_data[2]*_gyro_scale);

  // filter gyro data for angular rate measurements
  _current_state.p = (_current_state.p*(1000-_params.values[PARAM_GYRO_LPF_ALPHA]) + meas_p*(_params.values[PARAM_GYRO_LPF_ALPHA]))/1000;
  _current_state.q = (_current_state.q*(1000-_params.values[PARAM_GYRO_LPF_ALPHA]) + meas_q*(_params.values[PARAM_GYRO_LPF_ALPHA]))/1000;
  _current_state.r = (_current_state.r*(1000-_params.values[PARAM_GYRO_LPF_ALPHA]) + meas_r*(_params.values[PARAM_GYRO_LPF_ALPHA]))/1000;

  // Perform the Complementary Filter (forgive the unit adjustments.  This was written with basically a lot of trial an error)
  // current_state angles are in urad
  _current_state.phi = (alpha*(_current_state.phi  + (meas_p*dt)/1000)/1000 + (1000-alpha)*acc_phi);
  _current_state.theta = (alpha*(_current_state.theta  + (meas_q*dt)/1000)/1000 + (1000-alpha)*acc_theta);
  _current_state.psi = _current_state.psi  + (meas_r*dt)/1000;

  // wrap psi because we don't actually get a measurement of it
  if (abs(_current_state.psi) > 3141593)
  {
    _current_state.psi += 6283185 * -1* sign(_current_state.psi);
  }
}
