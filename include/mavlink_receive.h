#pragma once

#include <stdint.h>

#include "mavlink.h"

// global variable declarations
extern mavlink_offboard_control_t _offboard_control;
extern uint32_t _offboard_control_time_;

// function declarations
void mavlink_receive(void);
