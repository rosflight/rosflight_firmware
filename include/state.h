#include <stdint.h>

struct state_t{
  int16_t p;
  int16_t q;
  int16_t r;

  int16_t phi;
  int16_t theta;
  int16_t psi;
};

// global variable declaration
extern state_t _current_state;

