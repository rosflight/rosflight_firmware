#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <breezystm32/breezystm32.h>
#include <turbotrig/turbovec.h>

#include "estimator.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_receive.h"
#include "mavlink_stream.h"
#include "mavlink_util.h"
#include "mode.h"
#include "param.h"
#include "sensors.h"
#include "controller.h"
#include "mixer.h"
#include "rc.h"

#define PI 3.14159

#define PF(a) ((int32_t)(a*1000.0))

void printvec(int32_t* v)
{
  printf("[%d, %d, %d]\n", v[0], v[1], v[2]);
}

void printquat(int32_t* q)
{
  printf("[%d, %d, %d, %d]\n", q[0], q[1], q[2], q[3]);
}

void pfvec(vector_t v)
{
  printf("[%d, %d, %d]\n", (int32_t)(v.x*1000000), (int32_t)(v.y*1000000), (int32_t)(v.z*1000000));
}

void pfquat(quaternion_t v)
{
  printf("[%d, %d, %d, %d]\n", (int32_t)(v.w*1000000), (int32_t)(v.x*1000000), (int32_t)(v.y*1000000), (int32_t)(v.z*1000000));
}


uint32_t counter = 0;
uint32_t average_time = 0;
uint32_t start_time = 0;

float angle;
float omega;
int32_t axis;
float amplitude;
float frequency;
vector_t bias;

void setup(void)
{
  delay(500);

  // initialize sensors (for scale factors)
  _accel_scale = 2394;
  _gyro_scale = 4;

  // Initialize Estimator
  init_params();
  init_estimator(false, false, true);

  // Initialize Test Parameters
  angle = 0.0f;
  omega = 0.0f;

  axis = 0;

  amplitude = 1.0f;
  frequency = 0.25f;
  start_time = micros();
  bias.x = 0.1;
  bias.y = 0.0;
  bias.z = 0.0;
}

void loop(void)
{
  /*********************/
  /***  Pre-Process ***/
  /*********************/
  // get loop time
  static uint32_t prev_time;
  static int32_t dt = 0;
  uint32_t now = micros();

  average_time+=dt;
  counter++;

  float t = (float)(now-start_time)/1000000.0f;
  angle = amplitude*sin(2*PI*frequency*t);
  omega = 2*PI*frequency*amplitude*cos(2*PI*frequency*t);


  _accel_data[0] = 0;
  _accel_data[1] = 0;
  _accel_data[2] = 0;

  float roll = 0;
  float pitch = 0;
  float yaw = 0;

  /*****************************/
  /*** Generate Known Signal ***/
  /*****************************/
  switch(axis)
  {
    case 0:
      _accel_data[1] = (int32_t)(((9806.6)*1000.0*sin(angle))/_accel_scale);
      _accel_data[2] = (int32_t)(((9806.6)*1000.0*cos(angle))/_accel_scale);
      _gyro_data[0] = (int32_t)(((omega+bias.x)*1000.0)/_gyro_scale);
      _gyro_data[1] = (int32_t)(((bias.y)*1000.0)/_gyro_scale);
      _gyro_data[2] = (int32_t)(((bias.z)*1000.0)/_gyro_scale);
      roll = angle;
      break;
    case 1:
      _accel_data[0] = -1*(int32_t)(((9806.6)*1000.0*sin(angle))/_accel_scale);
      _accel_data[2] = (int32_t)(((9806.6)*1000.0*cos(angle))/_accel_scale);
      _gyro_data[0] = (int32_t)(((bias.x)*1000.0)/_gyro_scale);
      _gyro_data[1] = (int32_t)(((omega+bias.y)*1000.0)/_gyro_scale);
      _gyro_data[2] = (int32_t)(((bias.z)*1000.0)/_gyro_scale);
      pitch = angle;
      break;
    case 2:
      _accel_data[2] = (int32_t)(((9806.6)*1000.0)/_accel_scale);
      _gyro_data[0] = (int32_t)(((bias.x)*1000.0)/_gyro_scale);
      _gyro_data[1] = (int32_t)(((bias.y)*1000.0)/_gyro_scale);
      _gyro_data[2] = (int32_t)(((omega+bias.z)*1000.0)/_gyro_scale);
      yaw = angle;
      break;
    default:
      printf("bad axis");
      break;
  }

  if(now - prev_time > 10000)
  {
    /*********************/
    /*** Run Estimator ***/
    /*********************/
    uint32_t before = micros();
    run_estimator(now); // 193 us (gyro only, float-based)
    uint32_t after = micros();
    printf("%d,\t%d,\t%d,\t", _current_state.phi, _current_state.theta, _current_state.psi);
    printf("%d,\t%d,\t%d,\t%d\n", PF(roll), PF(pitch), PF(yaw),after-before);
    prev_time = now;
  }



//  printf("ax = %d\tay = %d\taz = %d\tgx = %d\tgy = %d\tgz = %d\n",
//      _accel_data[0]*_accel_scale/1000,
//      _accel_data[1]*_accel_scale/1000,
//      _accel_data[2]*_accel_scale/1000,
//      _gyro_data[0]*_gyro_scale,
//      _gyro_data[1]*_gyro_scale,
//      _gyro_data[2]*_gyro_scale);

}



