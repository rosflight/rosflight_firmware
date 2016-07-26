#include <breezystm32/breezystm32.h>

#include "mux.h"


extern bool _calibrate_rc;
void init_rc(void);
bool receive_rc(uint32_t now);
