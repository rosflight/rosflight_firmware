#pragma once

#include <stdint.h>

// function declarations
void mavlink_stream(uint32_t time_us);

void mavlink_stream_set_heartbeat_period_us(uint32_t period);
void mavlink_stream_set_imu_period_us(uint32_t period);
