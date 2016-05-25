#include <stdint.h>
#include <stdbool.h>

typedef struct state_t{
  int32_t p;
  int32_t q;
  int32_t r;

  int32_t phi;
  int32_t theta;
  int32_t psi;
} state_t;

extern state_t _current_state;

void init_estimator();
void run_estimator(int32_t dt);

