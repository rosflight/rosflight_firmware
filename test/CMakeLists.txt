cmake_minimum_required(VERSION 2.6)

add_definitions(-std=c++11)

# Strict Compilation
set(CMAKE_CXX_FLAGS "-std=c++11 -pedantic -pedantic-errors -Werror -Wall -Wextra \
  -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 -Wlogical-op -Wmissing-include-dirs \
  -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wundef -Wunused -Wvariadic-macros \
  -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel"
  ${CMAKE_CXX_FLAGS})

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(../include)
include_directories(../lib)
include_directories(../comms/mavlink)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

set(ROSFLIGHT_SRC
    ../src/rosflight.cpp
    ../src/param.cpp
    ../src/sensors.cpp
    ../src/state_manager.cpp
    ../src/estimator.cpp
    ../src/nanoprintf.cpp
    ../src/controller.cpp
    ../src/comm_manager.cpp
    ../src/command_manager.cpp
    ../src/rc.cpp
    ../src/mixer.cpp
    ../comms/mavlink/mavlink.cpp
    ../lib/turbomath/turbomath.cpp
    )



add_executable(unit_tests
        ${ROSFLIGHT_SRC}
        common.h
        common.cpp
        command_manager_test.cpp
        test_board.cpp
        turbotrig_test.cpp
        state_machine_test.cpp
        command_manager_test.cpp
        estimator_test.cpp
        parameters_test.cpp
        )
target_link_libraries(unit_tests ${GTEST_LIBRARIES} pthread)
