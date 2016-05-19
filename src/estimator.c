
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

state_t run_estimator(int16_t dt){
  int32_t prev_phi = _current_state.phi;
  int32_t prev_theta =_current_state.theta;

  int32_t alpha = 980;

  acc_phi = turboatan2(_acc)

  _current_state.phi = alpha*(prev_phi + (_current_state.p*dt)/1000) + (1-alpha)*
}
