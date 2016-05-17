#pragma once

#include "mavlink.h"

void mavlink_handle_msg_param_request_list(void);
void mavlink_handle_msg_param_request_read(const mavlink_message_t * const msg);
void mavlink_handle_msg_param_set(const mavlink_message_t * const msg);
void mavlink_send_next_param(void);
