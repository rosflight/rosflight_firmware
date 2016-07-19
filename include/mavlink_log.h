#pragma once

#include <breezystm32/breezystm32.h>

#include "mavlink.h"

#define mavlink_log(severity, format, ...) do { \
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN]; \
  sprintf(text, format, ##__VA_ARGS__); \
  mavlink_msg_statustext_send(MAVLINK_COMM_0, severity, text); \
  } while (0)


#define mavlink_log_critical(format, ...) mavlink_log(MAV_SEVERITY_CRITICAL, format, ##__VA_ARGS__)
#define mavlink_log_error(format, ...)    mavlink_log(MAV_SEVERITY_ERROR,    format, ##__VA_ARGS__)
#define mavlink_log_warning(format, ...)  mavlink_log(MAV_SEVERITY_WARNING,  format, ##__VA_ARGS__)
#define mavlink_log_info(format, ...)     mavlink_log(MAV_SEVERITY_INFO,     format, ##__VA_ARGS__)


#define mavlink_log_critical_throttle(delay_ms, format, ...) \
  do\
  {\
    static uint32_t last_hit = 0; \
    uint32_t now = millis(); \
    if (now - last_hit > delay_ms) \
    {\
      last_hit = now; \
      mavlink_log_critical(format, ##__VA_ARGS__); \
    }\
  } while(0)

#define mavlink_log_error_throttle(delay_ms, format, ...) \
  do\
  {\
    static uint32_t last_hit = 0; \
    uint32_t now = millis(); \
    if (now - last_hit > delay_ms) \
    {\
      last_hit = now; \
      mavlink_log_error(format, ##__VA_ARGS__); \
    }\
  } while(0)

#define mavlink_log_warning_throttle(delay_ms, format, ...) \
  do\
  {\
    static uint32_t last_hit = 0; \
    uint32_t now = millis(); \
    if (now - last_hit > delay_ms) \
    {\
      last_hit = now; \
      mavlink_log_warning(format, ##__VA_ARGS__); \
    }\
  } while(0)

#define mavlink_log_info_throttle(delay_ms, format, ...) \
  do\
  {\
    static uint32_t last_hit = 0; \
    uint32_t now = millis(); \
    if (now - last_hit > delay_ms) \
    {\
      last_hit = now; \
      mavlink_log_info(format, ##__VA_ARGS__); \
    }\
  } while(0)

