#include <stdlib.h>
#include <math.h>

#include <breezystm32/breezystm32.h>
#include <turbotrig/turbotrig.h>
#include <turbotrig/turbovec.h>

#include "sensors.h"
#include "param.h"

#include "estimator.h"

state_t _current_state;

static vector_t w1;
static vector_t w2;
static vector_t w;
static vector_t wbar;
static vector_t wfinal;
static vector_t w_meas;
static vector_t a;
static vector_t g;
static vector_t b;
static quaternion_t q_tilde;
static quaternion_t q_hat;

static void pfvec(vector_t v)
{
  printf("[%d, %d, %d]\n", (int32_t)(v.x*1000), (int32_t)(v.y*1000), (int32_t)(v.z*1000));
}

static void pfquat(quaternion_t v)
{
  printf("[%d, %d, %d, %d]\n", (int32_t)(v.w*1000), (int32_t)(v.x*1000), (int32_t)(v.y*1000), (int32_t)(v.z*1000));
}


void init_estimator()
{
  _current_state.p = 0;
  _current_state.q = 0;
  _current_state.r = 0;
  _current_state.phi = 0;
  _current_state.theta = 0;
  _current_state.psi = 0;

  q_hat.w = 1.0f;
  q_hat.x = 0.0f;
  q_hat.y = 0.0f;
  q_hat.z = 0.0f;

  w1.x = 0.0f;
  w1.y = 0.0f;
  w1.z = 0.0f;

  w2.x = 0.0f;
  w2.y = 0.0f;
  w2.z = 0.0f;

  b.x = 0.096;
  b.y = -0.008;
  b.z = -0.028;
}


void run_estimator(int32_t now)
{
  static int32_t last_time = 0;
  int32_t dt = now - last_time;
  last_time = now;

  w_meas.x = 0.0f;
  w_meas.y = 0.0f;
  w_meas.z = 0.0f;

  q_tilde.w = 1.0f;
  q_tilde.x = 0.0f;
  q_tilde.y = 0.0f;
  q_tilde.z = 0.0f;

  w.x = ((float)(_gyro_data[0]*_gyro_scale))/1000.0f;
  w.y = ((float)(_gyro_data[1]*_gyro_scale))/1000.0f;
  w.z = ((float)(_gyro_data[2]*_gyro_scale))/1000.0f;

  // this integration step adds 21 us
  wbar = vector_add(vector_add(scalar_multiply(-1.0f/12.0f,w2), scalar_multiply(8.0f/12.0f,w1)), scalar_multiply(5.0f/12.0f,w));
  w2 = w1;
  w1 = w;

  wfinal = vector_sub(wbar, b);

  float norm_w = sqrt(sqrd_norm(wfinal));

  if(norm_w > 0.0f)
  // Matrix Exponential Approximation (From Attitude Representation and Kinematic
  // Propagation for Low-Cost UAVs by Robert T. Casey e
  {
    float p = wfinal.x;
    float q = wfinal.y;
    float r = wfinal.z;
    quaternion_t qhat_np1;
    float t1 = cos((norm_w*dt)/(2000000.0f));
    float t2 = 1.0/norm_w * sin((norm_w*dt)/(2000000.0f));
    qhat_np1.w = t1*q_hat.w   - t2*p*q_hat.x - t2*q*q_hat.y - t2*r*q_hat.z;
    qhat_np1.x = t2*p*q_hat.w + t1*q_hat.x   + t2*r*q_hat.y - t2*q*q_hat.z;
    qhat_np1.y = t2*q*q_hat.w - t2*r*q_hat.x + t1*q_hat.y   + t2*p*q_hat.z;
    qhat_np1.z = t2*r*q_hat.w + t2*q*q_hat.x - t2*p*q_hat.y + t1*q_hat.z;
    q_hat = quaternion_normalize(qhat_np1);
  }

  // Extract Euler Angles
  _current_state.phi = turboatan2((int32_t)(2000*(q_hat.w*q_hat.x + q_hat.y*q_hat.z)),
                                  1000 - (int32_t)(2000*(q_hat.x*q_hat.x + q_hat.y*q_hat.y)));
  _current_state.theta = turboasin((int32_t)(2000*(q_hat.w*q_hat.y - q_hat.z*q_hat.x)));
  _current_state.psi = turboatan2((int32_t)(2000*(q_hat.w*q_hat.z + q_hat.x*q_hat.y)),
                                  1000 - (int32_t)(2000*(q_hat.y*q_hat.y + q_hat.z*q_hat.z)));
//  printf("roll = %d, pitch = %d, yaw = %d\n", _current_state.phi, _current_state.theta, _current_state.psi);
}

