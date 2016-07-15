#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

#include "mux.h"
#include "param.h"


void run_controller(uint32_t now);
control_t altitude_controller(control_t altitude_command);
control_t attitude_controller(control_t attitude_command, uint32_t now);
control_t rate_controller(control_t rate_command, uint32_t now);


#ifdef __cplusplus
}
#endif
