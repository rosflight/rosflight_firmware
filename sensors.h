#pragma once

#include <stdint.h>

// type definitions
typedef struct
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
} imuData_t;

// global variable declarations
extern imuData_t _imu_data;

// function declarations
void init_sensors(void);
void update_sensors(uint32_t time_us);
