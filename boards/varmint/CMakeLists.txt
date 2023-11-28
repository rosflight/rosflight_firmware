### project settings ###

set(CMAKE_SYSTEM_NAME Generic)
cmake_minimum_required(VERSION 3.8)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(varmint C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)


### include & source files ###

set(ROSFLIGHT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/../..)

include_directories(
    # Varmint
    include
    include/board

    # STM32
    lib/usb_device/App
    lib/usb_device/Target
    lib/drivers/STM32H7xx_HAL_Driver/Inc
    lib/drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    lib/drivers/CMSIS/Device/ST/STM32H7xx/Include
    lib/drivers/CMSIS/Include
    lib/middleware/ST/STM32_USB_Device_Library/Core/Inc
    lib/middleware/ST/STM32_USB_Device_Library/Class/CDC/Inc

    # ROSflight
    ${ROSFLIGHT_SOURCE_DIR}/include
    ${ROSFLIGHT_SOURCE_DIR}/include/interface
    ${ROSFLIGHT_SOURCE_DIR}/lib

    # MAVLink
    ${ROSFLIGHT_SOURCE_DIR}/comms/mavlink
    ${ROSFLIGHT_SOURCE_DIR}/comms/mavlink/v1.0
    ${ROSFLIGHT_SOURCE_DIR}/comms/mavlink/v1.0/common
    ${ROSFLIGHT_SOURCE_DIR}/comms/mavlink/v1.0/rosflight
)

file(GLOB_RECURSE SOURCES
    # Varmint
    "src/*.*"

    # STM32
    "lib/usb_device/*.c"
    "lib/drivers/*.c"
    "lib/middleware/*.c"

    # ROSflight
    "${ROSFLIGHT_SOURCE_DIR}/src/*.cpp"
    "${ROSFLIGHT_SOURCE_DIR}/lib/turbomath/turbomath.cpp"

    # MAVLink
    "${ROSFLIGHT_SOURCE_DIR}/comms/mavlink/mavlink.cpp"
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H753VIHX_FLASH.ld)
add_link_options(-T ${LINKER_SCRIPT})


### git information ###

execute_process(COMMAND git rev-parse --short=8 HEAD
        OUTPUT_VARIABLE GIT_VERSION_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY ${ROSFLIGHT_SOURCE_DIR})

execute_process(COMMAND git describe --tags --abbrev=8 --always --dirty --long
        OUTPUT_VARIABLE GIT_VERSION_STRING
        OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY ${ROSFLIGHT_SOURCE_DIR})

if("${GIT_VERSION_STRING}" STREQUAL "")
  set(GIT_VERSION_STRING "undefined")
endif()

if("${GIT_VERSION_HASH}" STREQUAL "")
  set(GIT_VERSION_HASH "0")
endif()


### preprocessor, compiler, linker options ###

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H753xx)

add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()


### build target ###

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
target_compile_definitions(${PROJECT_NAME}.elf PUBLIC
  GIT_VERSION_HASH=0x${GIT_VERSION_HASH}
  GIT_VERSION_STRING=\"${GIT_VERSION_STRING}\"
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")

