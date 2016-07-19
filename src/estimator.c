#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <breezystm32/breezystm32.h>
#include <turbotrig/turbotrig.h>
#include <turbotrig/turbovec.h>

#include "sensors.h"
#include "param.h"

#include "estimator.h"

state_t _current_state;
vector_t _adaptive_gyro_bias;

static vector_t w1;
static vector_t w2;
static vector_t wbar;
static vector_t wfinal;
static vector_t w_acc;
static const vector_t g = {0.0f, 0.0f, 1.0f};
static vector_t b;
static quaternion_t q_tilde;
static quaternion_t q_hat;
static int32_t last_time;

static bool mat_exp;
static bool quad_int;
static bool use_acc;

static float kp_;
static float ki_;
static uint32_t init_time;


void init_estimator(bool use_matrix_exponential, bool use_quadratic_integration, bool use_accelerometer)
{
  _current_state.p = 0.0f;
  _current_state.q = 0.0f;
  _current_state.r = 0.0f;
  _current_state.phi = 0.0f;
  _current_state.theta = 0.0f;
  _current_state.psi = 0.0f;

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

  kp_ = get_param_float(PARAM_FILTER_KP);
  ki_ = get_param_float(PARAM_FILTER_KI);
  init_time = _params.values[PARAM_INIT_TIME]*1000; // microseconds

  w_acc.x = 0.0f;
  w_acc.y = 0.0f;
  w_acc.z = 0.0f;

  q_tilde.w = 1.0f;
  q_tilde.x = 0.0f;
  q_tilde.y = 0.0f;
  q_tilde.z = 0.0f;

  mat_exp = use_matrix_exponential;
  quad_int = use_quadratic_integration;
  use_acc = use_accelerometer;

  _adaptive_gyro_bias.x = 0;
  _adaptive_gyro_bias.y = 0;
  _adaptive_gyro_bias.z = 0;

  last_time = 0;
}


void run_estimator(uint32_t now)
{
  static float kp, ki;
  if (last_time == 0)
  {
    last_time = now;
    return;
  }
  float dt = (now - last_time) * 1e-6f;
  last_time = now;

  // Crank up the gains for the first few seconds for quick convergence
  if (now < init_time)
  {
    kp = kp_*10.0f;
    ki = ki_*10.0f;
  }
  else
  {
    kp = kp_;
    ki = ki_;
  }

  // add in accelerometer
  float a_sqrd_norm = _accel.x*_accel.x + _accel.y*_accel.y + _accel.z*_accel.z;

  if (use_acc && a_sqrd_norm < 1.15f*1.15f*9.80665f*9.80665f && a_sqrd_norm > 0.85f*0.85f*9.80665f*9.80665f)
  {
    // Get error estimated by accelerometer measurement
    vector_t a = vector_normalize(_accel);
    // Get the quaternion from accelerometer (low-frequency measure q)
    // (Not in either paper)
    quaternion_t q_acc_inv = quaternion_inverse(quat_from_two_vectors(a, g));
    // Get the error quaternion between observer and low-freq q
    // Below Eq. 45 Mahoney Paper
    q_tilde = quaternion_multiply(q_acc_inv, q_hat);
    // Correction Term of Eq. 47a and 47b Mahoney Paper
    // w_acc = 2*s_tilde*v_tilde
    w_acc.x = -2.0f*q_tilde.w*q_tilde.x;
    w_acc.y = -2.0f*q_tilde.w*q_tilde.y;
    w_acc.z = -2.0f*q_tilde.w*q_tilde.z;

    // integrate biases from accelerometer feedback
    // (eq 47b Mahoney Paper, using correction term w_acc found above)
    b.x -= ki*w_acc.x*dt;
    b.y -= ki*w_acc.y*dt;
    b.z -= ki*w_acc.z*dt;
  }
  else
  {
    w_acc.x = 0.0f;
    w_acc.y = 0.0f;
    w_acc.z = 0.0f;
  }

  // Pull out Gyro measurements
  if (quad_int)
  {
    // Quadratic Integration (Eq. 14 Casey Paper)
    // this integration step adds 12 us on the STM32F10x chips
    wbar = vector_add(vector_add(scalar_multiply(-1.0f/12.0f,w2), scalar_multiply(8.0f/12.0f,w1)),
                      scalar_multiply(5.0f/12.0f,_gyro));
    w2 = w1;
    w1 = _gyro;
  }
  else
  {
    wbar = _gyro;
  }

  // Build the composite omega vector for kinematic propagation
  // This the stuff inside the p function in eq. 47a - Mahoney Paper
  wfinal = vector_add(vector_sub(wbar, b), scalar_multiply(kp, w_acc));

  // Propagate Dynamics (only if we've moved)
  float sqrd_norm_w = sqrd_norm(wfinal);
  if (sqrd_norm_w > 0.0f)
  {
    float p = wfinal.x;
    float q = wfinal.y;
    float r = wfinal.z;

    if (mat_exp)
    {
      // Matrix Exponential Approximation (From Attitude Representation and Kinematic
      // Propagation for Low-Cost UAVs by Robert T. Casey)
      // (Eq. 12 Casey Paper)
      // This adds 66 us on STM32F10x chips
      float norm_w = sqrt(sqrd_norm_w);
      quaternion_t qhat_np1;
      float t1 = cos((norm_w*dt)/2.0f);
      float t2 = 1.0f/norm_w * sin((norm_w*dt)/2.0f);
      qhat_np1.w = t1*q_hat.w   + t2*(          - p*q_hat.x - q*q_hat.y - r*q_hat.z);
      qhat_np1.x = t1*q_hat.x   + t2*(p*q_hat.w             + r*q_hat.y - q*q_hat.z);
      qhat_np1.y = t1*q_hat.y   + t2*(q*q_hat.w - r*q_hat.x             + p*q_hat.z);
      qhat_np1.z = t1*q_hat.z   + t2*(r*q_hat.w + q*q_hat.x - p*q_hat.y);
      q_hat = quaternion_normalize(qhat_np1);
    }
    else
    {
      // Euler Integration
      // (Eq. 47a Mahoney Paper), but this is pretty straight-forward
      quaternion_t qdot = {0.5f * (           - p*q_hat.x - q*q_hat.y - r*q_hat.z),
                           0.5f * ( p*q_hat.w             + r*q_hat.y - q*q_hat.z),
                           0.5f * ( q*q_hat.w - r*q_hat.x             + p*q_hat.z),
                           0.5f * ( r*q_hat.w + q*q_hat.x - p*q_hat.y)
                          };
      q_hat.w += qdot.w*dt;
      q_hat.x += qdot.x*dt;
      q_hat.y += qdot.y*dt;
      q_hat.z += qdot.z*dt;
      q_hat = quaternion_normalize(q_hat);
    }
  }

  // Extract Euler Angles for controller
  euler_from_quat(q_hat, &_current_state.phi, &_current_state.theta, &_current_state.psi);

  // Save off gyro
  wbar = vector_sub(wbar, b);
  float alpha = 0.98f;
  _current_state.p = (1.0f-alpha)*wbar.x + alpha*_current_state.p;
  _current_state.q = (1.0f-alpha)*wbar.y + alpha*_current_state.q;
  _current_state.r = (1.0f-alpha)*wbar.z + alpha*_current_state.r;

  // Save gyro biases for streaming to computer
  _adaptive_gyro_bias.x = b.x;
  _adaptive_gyro_bias.y = b.y;
  _adaptive_gyro_bias.z = b.z;
}

#ifdef __cplusplus
}
#endif

