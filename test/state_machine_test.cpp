#include <gtest/gtest.h>

#include "rosflight.h"
#include "test_board.h"

using namespace rosflight_firmware;

TEST(turbotrig_test, atan2_test) {
  testBoard board;
  ROSflight firmware(board);
  firmware.rosflight_init();
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
