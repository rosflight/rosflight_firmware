#pragma once

#include "mavlink.h"

void handle_param_set_msg(const mavlink_message_t * const msg);
void start_send_param_list(void);
void mavlink_send_low_priority(void);
