
#include <breezystm32/drv_mpu6050.h>

#include "sensors.h"
#include "state.h"
#include "estimator.h"

state_t init_estimator(){
  state_t state;
  state.p = 0;
  state.q = 0;
  state.r = 0;
  state.phi = 0;
  state.theta = 0;
  state.psi = 0;
  return state;
}

state_t run_estimator(int16_t dt){
  prev_phi = _current_state.phi;
  prev_theta =_current_state.theta;
}
