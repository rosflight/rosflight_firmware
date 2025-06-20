### project settings ###

# Building the projects:
# mkdir varmint_build      && cd varmint_build      && cmake .. -DBOARD_TO_BUILD=varmint      && make -j
# mkdir pixracer_pro_build && cd pixracer_pro_build && cmake .. -DBOARD_TO_BUILD=pixracer_pro && make -j
#
# alternate for make -j above is cmake --build . -j (note only one .)
# make -j OR cmake --build . -j
# to clean:
# make clean -j  OR  make --build . --target clean -j
#
# unit test:
# build:
# mkdir test_build         && cd test_build         && cmake .. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j
# test:
# ./test/unit_tests

cmake_minimum_required(VERSION 3.8)
project(rosflight_firmware C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

### git ###

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
)

file(GLOB_RECURSE ROSFLIGHT_SOURCES
    "src/*.cpp"
    "lib/turbomath/turbomath.cpp"
    "comms/mavlink/mavlink.cpp"
)

### select boards to compile ###

set(BOARD_TO_BUILD "" CACHE STRING "Select the board to build")
if("${BOARD_TO_BUILD}" STREQUAL "varmint")
    message("===== Selecting varmint board target. =====")
    set(TARGET_NAME "varmint_10X")
    add_subdirectory(boards/varmint_h7/varmint_10X)
elseif("${BOARD_TO_BUILD}" STREQUAL "pixracer_pro")
    message("===== Selecting pixracer pro board target. =====")
    set(TARGET_NAME "pixracer_pro")
    add_subdirectory(boards/varmint_h7/pixracer_pro)
elseif("${BOARD_TO_BUILD}" STREQUAL "test")
    message("===== Selecting test board target. ======")
    add_subdirectory(test)
else()
    message(FATAL_ERROR "No valid board selected! Please select the varmint, pixracer pro, or test board with -DBOARD_TO_BUILD=varmint, -DBOARD_TO_BUILD=pixracer_pro, OR -DBOARD_TO_BUILD=test.")
endif()
