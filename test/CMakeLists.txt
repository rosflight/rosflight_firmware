cmake_minimum_required(VERSION 2.6)
project(rosflight)

# C++ standard
add_definitions(-std=c++11)

# Strict Compilation
set(CMAKE_CXX_FLAGS "-pedantic -pedantic-errors -Werror -Wall -Wextra \
  -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 -Wlogical-op -Wmissing-include-dirs \
  -Wredundant-decls  -Wstrict-overflow=5 -Wundef -Wunused -Wvariadic-macros -Wold-style-cast -Wshadow \
  -Wctor-dtor-privacy -Wnoexcept -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel"
  ${CMAKE_CXX_FLAGS})



# Git information
execute_process(COMMAND git rev-parse --short=8 HEAD OUTPUT_VARIABLE GIT_VERSION_HASH OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long OUTPUT_VARIABLE GIT_VERSION_STRING OUTPUT_STRIP_TRAILING_WHITESPACE)

add_definitions(-DGIT_VERSION_HASH=0x${GIT_VERSION_HASH})
add_definitions(-DGIT_VERSION_STRING=\"${GIT_VERSION_STRING}\")

find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  rosbag
)

include_directories(${GTEST_INCLUDE_DIRS}
                    ../include
                    ../lib
                    ../comms/mavlink
                    ${EIGEN3_INCLUDE_DIRS}
                    /usr/include/eigen3
                    ${catkin_INCLUDE_DIRS})

# Create a library of the rosflight core flight stack
add_library(rosflight_core
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

# Build the unit tests
add_executable(unit_tests
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
target_link_libraries(unit_tests ${GTEST_LIBRARIES} pthread rosflight_core)

# Create the rosbag parser
add_executable(rosbag_parser rosbag_parser.cpp test_board.cpp)
target_link_libraries(rosbag_parser ${catkin_LIBRARIES} rosflight_core ${YAML_CPP_LIBRARIES})


