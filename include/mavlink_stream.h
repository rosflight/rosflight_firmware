#pragma once

#include <stdint.h>

// type definitions
typedef enum
{
  MAVLINK_STREAM_ID_HEARTBEAT,

  MAVLINK_STREAM_ID_ATTITUDE,

  MAVLINK_STREAM_ID_IMU,
  MAVLINK_STREAM_ID_DIFF_PRESSURE,
  MAVLINK_STREAM_ID_BARO,
  MAVLINK_STREAM_ID_SONAR,

  MAVLINK_STREAM_ID_SERVO_OUTPUT_RAW,
  MAVLINK_STREAM_ID_RC_RAW,
  MAVLINK_STREAM_ID_LOW_PRIORITY,
  MAVLINK_STREAM_COUNT
} mavlink_stream_id_t;

// function declarations
void mavlink_stream(uint32_t time_us);
void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate);
void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us);
