### project settings ###
cmake_minimum_required(VERSION 3.8)

add_definitions(-DBUILD_PIXRACER_PRO)
add_definitions(-DSTM32H743xx)
include("${CMAKE_CURRENT_SOURCE_DIR}/../common/CMakeListsCommon.cmake")