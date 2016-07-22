#include <breezystm32/breezystm32.h>

#include "mux.h"

typedef struct
{
  int16_t channel;
  int16_t direction;
} rc_switch_t;

void init_rc(void);
bool rc_switch(int16_t channel);
bool receive_rc(uint32_t now);
