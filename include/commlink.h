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
    virtual void init(Board* _board, Params* _params, Sensors* _sensors) = 0;
    virtual void receive() = 0;
    virtual void stream() = 0;
    virtual void update_param(uint16_t param_id) = 0;
    virtual void set_streaming_rate(uint16_t param_id, int32_t rate) = 0;
};


} //namespace
