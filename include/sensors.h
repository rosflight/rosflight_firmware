#pragma once

#include <stdint.h>
#include <stdbool.h>

// global variable declarations
extern int16_t _accel_data[3];
extern int16_t _gyro_data[3];
extern int32_t _accel_scale; // converts to mm/s^2
extern int32_t _gyro_scale; // converts to urad/s

extern bool _diff_pressure_present;
extern int16_t _diff_pressure;
extern int16_t _temperature;

// function declarations
void init_sensors(void);
bool update_sensors(uint32_t time_us);
