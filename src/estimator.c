
#include <breezystm32/drv_mpu6050.h>
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

void run_estimator(int32_t dt){
  int32_t tau = 100; // desired time constant of the filter (us) <-- should be a param
  int32_t alpha = (1000000*tau)/(tau*1000+5000);

  // pull in accelerometer data
  int32_t acc_phi = turboatan2(_accel_data[1], _accel_data[2]);
  int32_t acc_theta = turboatan2(_accel_data[0], _accel_data[2]);

  // pull in gyro data
  _current_state.p = _gyro_data[0];
  _current_state.q = _gyro_data[1];
  _current_state.r = _gyro_data[2];

  _current_state.phi = ((alpha*(_current_state.phi + (_current_state.p*dt)/1000000)) + (1000-alpha)*acc_phi)/1000;
  _current_state.theta = ((alpha*(_current_state.theta + (_current_state.q*dt)/1000000)) + (1000-alpha)*acc_theta)/1000;
  _current_state.psi = _current_state.psi + (_current_state.r*dt)/1000000;

  // wrap psi because we don't actually get a measurement of it
  if(abs(_current_state.psi) > 3142){
    _current_state.psi += 6284 * -1* sign(_current_state.psi);
  }
}
