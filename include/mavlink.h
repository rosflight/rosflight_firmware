#pragma once

#include <mavlink/v1.0/mavlink_types.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
extern mavlink_system_t mavlink_system;
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#include <mavlink/v1.0/rosflight/mavlink.h>

// function declarations
void init_mavlink(void);
