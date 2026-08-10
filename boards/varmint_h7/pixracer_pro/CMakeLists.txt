# This is not a top-level project.
# It is called from rosflight_firmware/CMakeLists.txt, which defines the
# rosflight_firmware::rosflight_firmware library that is linked inside of
# add_stm32_h7_board.

include("${CMAKE_CURRENT_LIST_DIR}/../common/stm32_h7_board.cmake")
add_stm32_h7_board(pixracer_pro STM32H743xx)
