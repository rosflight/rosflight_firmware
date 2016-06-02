#include <stdint.h>
#include <stdbool.h>

#include "mux.h"
#include "param.h"


control_t altitude_controller(control_t altitude_command);
control_t attitude_controller(control_t attitude_command);
control_t rate_controller(control_t rate_command);


