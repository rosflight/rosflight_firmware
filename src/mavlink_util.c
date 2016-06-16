#include <breezystm32/breezystm32.h>

#include "mavlink.h"

void mavlink_send_named_value_int(const char *const name, int32_t value)
{
  mavlink_msg_named_value_int_send(MAVLINK_COMM_0, millis(), name, value);
}

void mavlink_send_named_value_float(const char *const name, float value)
{
  mavlink_msg_named_value_float_send(MAVLINK_COMM_0, millis(), name, value);
}
