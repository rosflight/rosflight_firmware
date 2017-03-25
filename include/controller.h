#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "mux.h"
#include "param.h"

void run_controller();
void init_controller();
void calculate_equilbrium_torque_from_rc();


#ifdef __cplusplus
}
#endif
