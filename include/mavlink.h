#pragma once

#include <mavlink/v1.0/mavlink_types.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
extern mavlink_system_t mavlink_system;
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#include <mavlink/v1.0/rosflight/mavlink.h>

// function declarations
void init_mavlink(void);

void send_heartbeat(void);
void send_imu(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
