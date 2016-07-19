#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <turbotrig/turbovec.h>

#include <stdint.h>
#include <stdbool.h>

// global variable declarations
extern vector_t _accel;
extern vector_t _gyro;
extern float _imu_temperature;
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
extern uint32_t _sonar_time;

// function declarations
void init_sensors(void);
bool update_sensors(uint32_t time_us);

bool calibrate_acc(void);
bool calibrate_gyro(void);
#ifdef __cplusplus
}
#endif
