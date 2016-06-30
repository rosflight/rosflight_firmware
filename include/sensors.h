#pragma once

#include <stdint.h>
#include <stdbool.h>

// global variable declarations
extern int16_t _accel_data[3];
extern int16_t _gyro_data[3];
extern int32_t _accel_scale; // converts to mm/s^2
extern int32_t _gyro_scale; // converts to urad/s
extern int16_t _imu_temperature;
extern uint32_t _imu_time;
extern bool _imu_ready;

extern bool _diff_pressure_present;
extern int16_t _diff_pressure;
extern int16_t _temperature;

extern bool _baro_present;
extern int16_t _baro_pressure;
extern int16_t _baro_temperature;

extern bool _sonar_present;
extern int16_t _sonar_range;

// function declarations
void init_sensors(void);
bool update_sensors(uint32_t time_us);
