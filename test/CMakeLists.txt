cmake_minimum_required(VERSION 2.6)

add_definitions(-std=c++11)
add_definitions(-DUSING_STDLIB_PRINTF)
 
# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
 
include_directories(../lib)
include_directories(../lib/turbotrig)
include_directories(../include)

# Link runTests with what we want to test and the GTest and pthread library
add_executable(sample sample.cpp)
target_link_libraries(sample ${GTEST_LIBRARIES} pthread)

# Turbotrig test
add_executable(turbotrig_test 
	../lib/turbotrig/turbotrig.c
	turbotrig_test.cpp)
target_link_libraries(turbotrig_test ${GTEST_LIBRARIES} pthread)


set(ROSFLIGHT_SRC
	../src/rosflight.cpp 
    ../src/param.cpp 
    ../src/sensors.cpp 
    ../src/state_manager.cpp 
    ../src/estimator.cpp 
    ../src/mavlink.cpp 
    ../src/controller.cpp 
    ../src/command_manager.cpp 
    ../src/rc.cpp 
    ../src/mixer.cpp
    ../lib/turbotrig/turbotrig.c
    ../lib/turbotrig/turbovec.c)

add_executable(state_machine_test 
        ${ROSFLIGHT_SRC}
        state_machine_test.cpp
	test_board.cpp)
target_link_libraries(state_machine_test ${GTEST_LIBRARIES} pthread)