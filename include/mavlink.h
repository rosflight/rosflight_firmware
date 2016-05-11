#pragma once

#include <stdint.h>

// function declarations
void send_heartbeat(void);
void send_imu(uint64_t time_usec, float ax, float ay, float az, float gx, float gy, float gz);
