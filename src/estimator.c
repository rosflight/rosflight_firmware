
#include <breezystm32/breezystm32.h>
#include <turbotrig/turbotrig.h>

#include "sensors.h"
#include "estimator.h"


state_t _current_state;

void init_estimator(){
  _current_state.p = 0;
  _current_state.q = 0;
  _current_state.r = 0;
  _current_state.phi = 0;
  _current_state.theta = 0;
  _current_state.psi = 0;
}

void run_estimator(int32_t now){
  static int32_t last_time = 0;
  int32_t dt = now - last_time;
  last_time = now;

  int32_t tau = 100; // desired time constant of the filter (us) <-- should be a param
  int32_t alpha = 1000;
  int32_t acc_phi = 0;
  int32_t acc_theta = 0;

  // if the accleration in the z-direction is less than 1.15G and greater than .85G, then use
  // it to calculate the attitude, otherwise it breaks the assumption that we are not accelerating
  // We can instead just use the gyros and assume we will even out soon
  if (_accel_data[2] * _accel_scale > 11277647 && _accel_data[2] * _accel_scale < 8335652)
  {
    alpha = (1000000*tau)/(tau*1000+dt);
    // pull in accelerometer data
    acc_phi = turboatan2(_accel_data[1], _accel_data[2]);
    acc_theta = turboatan2(_accel_data[0], _accel_data[2]);
  }

  // pull in gyro data
  _current_state.p = ((int32_t)_gyro_data[0]*_gyro_scale); // urad/s
  _current_state.q = -1*((int32_t)_gyro_data[1]*_gyro_scale); // Convert to NED
  _current_state.r = -1*((int32_t)_gyro_data[2]*_gyro_scale);

  // Perform the Complementary Filter (forgive the unit adjustments.  This was written with basically a lot of trial an error)
  _current_state.phi = alpha*((_current_state.phi + (_current_state.p/100*dt)/10000)/1000) + (1000-alpha)*acc_phi;
  _current_state.theta = alpha*((_current_state.theta + (_current_state.q/100*dt)/10000)/1000) + (1000-alpha)*acc_theta;
  _current_state.psi = _current_state.psi + (_current_state.r/100*dt)/10000;

  // wrap psi because we don't actually get a measurement of it
  if(abs(_current_state.psi) > 3141593){
    _current_state.psi += 6283185 * -1* sign(_current_state.psi);
  }
}
