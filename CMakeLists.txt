### project settings ###

# Tools for building the projects and unit tests
#   sudo apt install -y build-essential cmake gcc-arm-none-eabi ninja-build libgtest-dev
# also for gtest (symlink pre-built libs from libgtest-dev to usable location):
#   sudo ln -s /usr/lib/x86_64-linux-gnu/libgtest*.a /usr/lib/.
# OR to build gtest from source (check for pre-built files first):
#   cd /usr/src/gtest
#   sudo cmake -G Ninja CMakeLists.txt
#   sudo ninja
#   sudo cp ./lib/libgtest*.a /usr/lib

# Build Boards:
# cmake --preset varmint-10X-release
# cmake --build build/varmint-10X-release
# cmake --preset varmint-11X-release
# cmake --build build/varmint-11X-release
# cmake --preset pixracer-pro-release
# cmake --build build/pixracer-pro-release

# Build and run Unit Tests:
# cmake --preset test-release
# cmake --build build/test-release
# ./build/test-release/test/unit_tests

cmake_minimum_required(VERSION 3.20...3.28)

set(rosflight_firmware_is_top_level FALSE)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(rosflight_firmware_is_top_level TRUE)
  project(rosflight_firmware C CXX ASM)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

### git ###

# get version info
execute_process(COMMAND git rev-parse --short=8 HEAD
  OUTPUT_VARIABLE GIT_VERSION_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long
  OUTPUT_VARIABLE GIT_VERSION_STRING
  OUTPUT_STRIP_TRAILING_WHITESPACE
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
if("${GIT_VERSION_STRING}" STREQUAL "")
  set(GIT_VERSION_STRING "undefined")
endif()
if("${GIT_VERSION_HASH}" STREQUAL "")
  set(GIT_VERSION_HASH "0")
endif()

### rosflight_firmware libraries ###

add_library(eigen_local INTERFACE)
add_library(rosflight_firmware::eigen_local ALIAS eigen_local)
target_compile_features(eigen_local INTERFACE cxx_std_17)
# Marking as SYSTEM include suppresses warnings from Eigen
target_include_directories(eigen_local SYSTEM INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

add_library(turbomath lib/turbomath/turbomath.cpp)
add_library(rosflight_firmware::turbomath ALIAS turbomath)
target_compile_features(turbomath PUBLIC cxx_std_17)
target_include_directories(turbomath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(firmware_version INTERFACE)
add_library(rosflight_firmware::version ALIAS firmware_version)
target_compile_definitions(firmware_version INTERFACE
  GIT_VERSION_HASH=0x${GIT_VERSION_HASH}
  GIT_VERSION_STRING="${GIT_VERSION_STRING}"
)

add_library(mavlink_headers INTERFACE)
add_library(rosflight_firmware::mavlink_headers ALIAS mavlink_headers)
# Marking as SYSTEM include suppresses warnings from generated mavlink headers
target_include_directories(mavlink_headers SYSTEM INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/comms/mavlink/v1.0
)

add_library(rosflight_firmware
  ${CMAKE_CURRENT_SOURCE_DIR}/src/comm_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/command_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mixer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/param.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rosflight.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/state_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink_adapter.cpp
)
add_library(rosflight_firmware::rosflight_firmware ALIAS rosflight_firmware)
target_compile_features(rosflight_firmware PUBLIC cxx_std_17)
target_include_directories(rosflight_firmware PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/interface
)
target_link_libraries(rosflight_firmware
  PUBLIC
    rosflight_firmware::mavlink_headers
    rosflight_firmware::version
    rosflight_firmware::turbomath
  PRIVATE
    rosflight_firmware::eigen_local
)

# Create OBJECT library that separates out `int main()` to be included in the
# final <board_name>.elf target
add_library(rosflight_firmware_main OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)
add_library(rosflight_firmware::firmware_main ALIAS rosflight_firmware_main)
target_link_libraries(rosflight_firmware_main PUBLIC
  rosflight_firmware::rosflight_firmware
)

# Don't configure a board for SIL case (when included from rosflight_sim)
if(NOT rosflight_firmware_is_top_level)
  return()
endif()

### Create registry of boards with directory location for each
set(BOARD_TO_BUILD "" CACHE STRING "Select the board to build")
set(supported_boards
  varmint_10X
  varmint_11X
  pixracer_pro
  test
)
set_property(CACHE BOARD_TO_BUILD PROPERTY STRINGS ${supported_boards})

set(BOARD_DIR_varmint_10X "boards/varmint_h7/varmint_10X")
set(BOARD_DIR_varmint_11X "boards/varmint_h7/varmint_11X")
set(BOARD_DIR_pixracer_pro "boards/varmint_h7/pixracer_pro")

### select boards to compile ###
if("${BOARD_TO_BUILD}" STREQUAL "test")
  message("===== Selecting test board target. ======")
  target_compile_definitions(rosflight_firmware PRIVATE BUILD_TEST_BOARD)
  add_subdirectory(test)

elseif(DEFINED BOARD_DIR_${BOARD_TO_BUILD})
  set(board_dir "${CMAKE_CURRENT_SOURCE_DIR}/${BOARD_DIR_${BOARD_TO_BUILD}}")

  if(EXISTS "${board_dir}/CMakeLists.txt")
    message("===== Selecting ${BOARD_TO_BUILD} board target. ======")
    add_subdirectory("${board_dir}" "${CMAKE_CURRENT_BINARY_DIR}/output")
  else()
    message(FATAL_ERROR "Board '${BOARD_TO_BUILD}' is registered, "
      "but no CMakeLists.txt exists at: ${board_dir}"
    )
  endif()

else()
  list(JOIN supported_boards ", " supported_boards_message)
  message(FATAL_ERROR "No valid board selected! Received: '${BOARD_TO_BUILD}'\n"
    "Valid board options: ${supported_boards_message}\n"
    "Select board by using a CMake preset (mode can be 'debug' or 'release'):\n"
    "    cmake --preset <board_selection>-<mode>\n"
    "Example:\n"
    "    cmake --preset pixracer-pro-release\n"
    "    cmake --build build/pixracer-pro-release"
  )
endif()
