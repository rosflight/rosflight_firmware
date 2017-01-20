#ifndef RC_H_
#define RC_H_

#include <breezystm32/breezystm32.h>

#include "mux.h"

typedef struct
{
  int16_t channel;
  int16_t direction;
} rc_switch_t;

typedef enum
{
  PARALLEL_PWM,
  CPPM,
} rc_type_t;


extern bool _calibrate_rc;
void init_rc(void);
bool rc_switch(int16_t channel);
bool receive_rc(uint64_t now);

#endif
