#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-pedantic"
#include <mavlink/v1.0/mavlink_types.h>
#pragma GCC diagnostic pop

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
extern mavlink_system_t mavlink_system;
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

// this needs to be include after the above declarations
#include <mavlink/v1.0/rosflight/mavlink.h>

// function declarations
void init_mavlink(void);
