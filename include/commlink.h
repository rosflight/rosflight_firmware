#pragma once

#include <stdint.h>

#include "board.h"
#include "sensors.h"
#include "param.h"

namespace rosflight {

class Params;
class Sensors;

class CommLink {
public:

  enum
  {
    STREAM_ID_HEARTBEAT,

    STREAM_ID_ATTITUDE,

    STREAM_ID_IMU,
    STREAM_ID_DIFF_PRESSURE,
    STREAM_ID_BARO,
    STREAM_ID_SONAR,
    STREAM_ID_MAG,

    STREAM_ID_SERVO_OUTPUT_RAW,
    STREAM_ID_RC_RAW,
    STREAM_ID_LOW_PRIORITY,
    STREAM_COUNT
  };

  virtual void init(Board* _board, Params* _params, Sensors* _sensors) = 0;
  virtual void receive() = 0;
  virtual void stream() = 0;
  virtual void update_param(uint16_t param_id) = 0;
  virtual void set_streaming_rate(uint8_t stream_id, int32_t rate) = 0;
};


} //namespace
