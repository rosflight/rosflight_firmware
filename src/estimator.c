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
static vector_t w_acc;
static vector_t a;
static vector_t g;
static vector_t b;
static quaternion_t q_tilde;
static quaternion_t q_hat;

static float kp;
static float ki;

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

  b.x = 0.0f;
  b.y = 0.0f;
  b.z = 0.0f;

  g.x = 0.0f;
  g.y = 0.0f;
  g.z = 1.0f;

  kp = 1.0f;
  ki = 0.001f;
}


void run_estimator(int32_t now)
{
  static int32_t last_time = 0;
  int32_t dt = now - last_time;
  last_time = now;

  w_acc.x = 0.0f;
  w_acc.y = 0.0f;
  w_acc.z = 0.0f;

  q_tilde.w = 1.0f;
  q_tilde.x = 0.0f;
  q_tilde.y = 0.0f;
  q_tilde.z = 0.0f;

  // add in accelerometer
  a.x = ((float)(_accel_data[0]*_accel_scale))/1000000.0f;
  a.y = ((float)(_accel_data[1]*_accel_scale))/1000000.0f;
  a.z = ((float)(_accel_data[2]*_accel_scale))/1000000.0f;
  float a_sqrd_norm = a.x*a.x + a.y*a.y + a.z*a.z;

  if(a_sqrd_norm < 1.15*1.15*9.80665*9.80665 && a_sqrd_norm > 0.85*0.85*9.80665*9.80665)
  {
    a = vector_normalize(a);
    quaternion_t q_acc_inv = quaternion_inverse(quat_from_two_vectors(a, g));
//    printf("q_acc_inv = "); pfquat(q_acc_inv);
    int32_t roll, pitch, yaw;
    euler_from_quat(q_acc_inv, &roll, &pitch, &yaw);
    q_tilde = quaternion_multiply(q_acc_inv, q_hat);
//    printf("q_tilde = "); pfquat(q_tilde);
//    w_acc.x = -2.0f*q_tilde.w*q_tilde.x;
//    w_acc.y = -2.0f*q_tilde.w*q_tilde.y;
//    w_acc.z = -2.0f*q_tilde.w*q_tilde.z;
    euler_from_quat(q_tilde, &roll, &pitch, &yaw);
    printf("q_acc_inv: roll = %d\tpitch = %d\tyaw = %d\n", roll, pitch, yaw);
//    printf("wacc = ");pfvec(w_acc);
  }

  // integrate biases

  w.x = ((float)(_gyro_data[0]*_gyro_scale))/1000.0f;
  w.y = 0.0f;//((float)(_gyro_data[1]*_gyro_scale))/1000.0f;
  w.z = 0.0f;//((float)(_gyro_data[2]*_gyro_scale))/1000.0f;

  // this integration step adds 21 us
//  wbar = vector_add(vector_add(scalar_multiply(-1.0f/12.0f,w2), scalar_multiply(8.0f/12.0f,w1)), scalar_multiply(5.0f/12.0f,w));
//  w2 = w1;
//  w1 = w;
  wbar = w;

  wfinal = w; //vector_add(vector_sub(wbar, b), scalar_multiply(kp, w_acc));

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
  euler_from_quat(q_tilde, &_current_state.phi, &_current_state.theta, &_current_state.psi);
  printf("q_hat:    roll = %d\tpitch = %d\tyaw = %d\t", _current_state.phi, _current_state.theta, _current_state.psi);
  printf("omega = "); pfvec(w);
//  printf("kp = %d\tki = %d\taccel = ", (int32_t)(kp*1000), (int32_t)(ki*1000)); pfvec(a);
}

