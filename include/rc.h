#ifndef RC_H_
#define RC_H_

#include <breezystm32/breezystm32.h>

#include "mux.h"

typedef enum
{
  PARALLEL_PWM,
  CPPM,
} rc_type_t;


extern bool _calibrate_rc;
void init_rc(void);
bool receive_rc(uint32_t now);

#endif
