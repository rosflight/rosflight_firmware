#pragma once

#include <stdint.h>

// type definitions
// Needs to match mavlink_streams variable in mavlink_stream.c
typedef enum
{
  MAVLINK_STREAM_ID_HEARTBEAT,
  MAVLINK_STREAM_ID_STATUS,

  MAVLINK_STREAM_ID_ATTITUDE,
  MAVLINK_STREAM_ID_IMU,
  MAVLINK_STREAM_ID_DIFF_PRESSURE,
  MAVLINK_STREAM_ID_BARO,
  MAVLINK_STREAM_ID_SONAR,
  MAVLINK_STREAM_ID_MAG,
  MAVLINK_STREAM_ID_OUTPUT_RAW,
  MAVLINK_STREAM_ID_RC_RAW,

  MAVLINK_STREAM_ID_LOW_PRIORITY,

  MAVLINK_STREAM_COUNT
} mavlink_stream_id_t;

// function declarations
void mavlink_stream(uint64_t time_us);
void mavlink_stream_set_rate(mavlink_stream_id_t stream_id, uint32_t rate);
void mavlink_stream_set_period(mavlink_stream_id_t stream_id, uint32_t period_us);
