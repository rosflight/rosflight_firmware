#pragma once

#include <stdint.h>

#include "mavlink.h"

// global variable declarations
//mavlink_offboard_control_t mavlink_offboard_control;
extern uint32_t _offboard_control_time;

// function declarations
void mavlink_receive(void);
