#include "rosflight.h"

int main(void)
{
  rosflight_init();

  while(1)
  {
    rosflight_run();
  }
  return 0;
}
