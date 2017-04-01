#include "naze32.h"

#include "rosflight.h"
#include "mavlink.h"
//#include "param.h"
//#include "arming_fsm.h"


int main(void)
{
  rosflight::Naze32 board;
  board.init_board();

  rosflight::Mavlink mavlink;

  rosflight::ROSflight firmware(&board, &mavlink);
  firmware.rosflight_init();

  while(1)
  {
//    board.clock_delay(100);
//    board.led0_toggle();
    firmware.rosflight_run();
  }
  return 0;
}
