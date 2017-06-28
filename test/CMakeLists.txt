cmake_minimum_required(VERSION 2.6)
 
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
