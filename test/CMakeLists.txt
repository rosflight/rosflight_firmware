# This is not a top-level project.
# It is called from rosflight_firmware/CMakeLists.txt, which defines the
# rosflight_firmware libraries that are linked to the tests.

# Locate GTest
find_package(GTest REQUIRED)

set_property(SOURCE test_board.cpp APPEND PROPERTY COMPILE_OPTIONS
  "$<$<CXX_COMPILER_ID:GNU>:-Wno-unused-parameter>"
)
add_executable(unit_tests
  common.cpp
  test_board.cpp
  turbotrig_test.cpp
  state_machine_test.cpp
  command_manager_test.cpp
  estimator_test.cpp
  parameters_test.cpp
  sensors_battery_test.cpp
  rosflight_test.cpp
)

# Strict Compilation
target_compile_options(unit_tests PRIVATE
  -pedantic -pedantic-errors -Werror -Wall -Wextra -Wcast-align -Wcast-qual
  -Wdisabled-optimization -Wformat=2 -Wlogical-op -Wmissing-include-dirs
  -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wundef -Wunused
  -Wvariadic-macros -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast
  -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel
)

target_link_libraries(unit_tests PRIVATE
  rosflight_firmware::rosflight_firmware
  rosflight_firmware::eigen_local
  GTest::GTest
  GTest::Main
)
