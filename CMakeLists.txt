### project settings ###

cmake_minimum_required(VERSION 3.8)
project(rosflight_firmware C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

add_compile_options(-Wall)


### git ###

# clone mavlink submodule if it is missing
set(MAVLINK_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/comms/mavlink/v1.0")
if(NOT EXISTS "${MAVLINK_SUBMODULE_DIR}/.git")
    message(STATUS "MAVlink submodule not found at ${MAVLINK_SUBMODULE_DIR}")
    execute_process(
        COMMAND git submodule update --init --recursive
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

# get version info
execute_process(COMMAND git rev-parse --short=8 HEAD
        OUTPUT_VARIABLE GIT_VERSION_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long
        OUTPUT_VARIABLE GIT_VERSION_STRING
        OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
if("${GIT_VERSION_STRING}" STREQUAL "")
  set(GIT_VERSION_STRING "undefined")
endif()
if("${GIT_VERSION_HASH}" STREQUAL "")
  set(GIT_VERSION_HASH "0")
endif()


### source files ###

include_directories(
    include
    include/interface
    lib
    comms/mavlink
    comms/mavlink/v1.0
    comms/mavlink/v1.0/common
    comms/mavlink/v1.0/rosflight
)

file(GLOB_RECURSE ROSFLIGHT_SOURCES
    "src/*.cpp"
    "lib/turbomath/turbomath.cpp"
    "comms/mavlink/mavlink.cpp"
)


### select boards to compile ###

option(BUILD_VARMINT "Build the varmint board target" ON)

if(BUILD_VARMINT)
    message(STATUS "Selecting varmint board target")
    add_subdirectory(boards/varmint)
endif()

