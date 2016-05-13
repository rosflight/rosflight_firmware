#pragma once

#include <stdint.h>

#include <mavlink/v1.0/mavlink_types.h>

// global variables
extern mavlink_message_t _msgbuf; // this is the buffer for both sending and receiving

// function declarations
void init_mavlink(void);
void send_heartbeat(void);
void send_imu(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
