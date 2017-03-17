#ifndef RC_H_
#define RC_H_

#include <stdint.h>
#include <stdbool.h>

#include "param.h"

#include "mux.h"

//typedef struct
//{
//    int32_t channel_param;
//    int32_t max_angle_param;
//    int32_t max_rate_param;
//    int32_t center_param;
//    int32_t bottom_param;
//    int32_t range_param;
//    control_channel_t* control_channel_ptr;
//} rc_channel_t;

typedef enum
{
  RC_STICK_X,
  RC_STICK_Y,
  RC_STICK_Z,
  RC_STICK_F,
  RC_STICKS_COUNT
} rc_stick_t;

typedef enum
{
  RC_SWITCH_ARM,
  RC_SWITCH_ATT_OVERRIDE,
  RC_SWITCH_THROTTLE_OVERRIDE,
  RC_SWITCH_ATT_TYPE,
  RC_SWITCH_THROTTLE_TYPE,
  RC_SWITCHES_COUNT
} rc_switch_t;

typedef enum
{
  PARALLEL_PWM,
  CPPM,
} rc_type_t;


extern bool _calibrate_rc;
void init_rc(void);
float rc_stick(rc_stick_t channel);
bool rc_switch(rc_switch_t channel);
bool rc_switch_mapped(rc_switch_t channel);
bool receive_rc();

bool rc_low(int16_t channel);
bool rc_high(int16_t channel);

#endif
