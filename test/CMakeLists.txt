cmake_minimum_required(VERSION 2.6)

add_definitions(-std=c++11)

# Strict Compilation
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wstrict-aliasing -pedantic -fmax-errors=5 \
  -Werror -Wunreachable-code -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization \
  -Wformat=2 -Winit-self -Wlogical-op -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual \
  -Wredundant-decls -Wshadow -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default -Wundef \
  -Wno-unused -Wno-variadic-macros -Wno-parentheses -fdiagnostics-show-option ${CMAKE_CXX_FLAGS}")

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

include_directories(../include)
include_directories(../lib)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

set(ROSFLIGHT_SRC
    ../src/rosflight.cpp
    ../src/param.cpp
    ../src/sensors.cpp
    ../src/state_manager.cpp
    ../src/estimator.cpp
    ../src/mavlink.cpp
    ../src/nanoprintf.cpp
    ../src/controller.cpp
    ../src/command_manager.cpp
    ../src/rc.cpp
    ../src/mixer.cpp
    ../lib/turbotrig/turbotrig.cpp
    ../lib/turbotrig/turbovec.cpp
    )

add_executable(unit_tests
        ${ROSFLIGHT_SRC}
        command_manager_test.cpp
        test_board.cpp
        turbotrig_test.cpp
        state_machine_test.cpp
        command_manager_test.cpp
        estimator_test.cpp
        )
target_link_libraries(unit_tests ${GTEST_LIBRARIES} pthread)
