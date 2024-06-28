### project settings ###

#cmake_minimum_required(VERSION 3.8)
set(CMAKE_SYSTEM_NAME Generic)

project(varmint C CXX ASM)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)

### source files ###

# Add test directory if being compiled without rosflight_firmware. Normally ROSFLIGHT_SOURCES is
#  provided by the CMakeLists.txt file in the root of the firmware repo and has the source files
#  for the firmware.
if(NOT DEFINED ROSFLIGHT_SOURCES)
    include_directories(../test)
    set(ROSFLIGHT_SOURCES "../test/main.cpp")
endif()

include_directories(
    Core/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/Include
    ../common
    ../common/drivers
    ../common/AL94_USB_Composite
    ../common/AL94_USB_Composite/App
    ../common/AL94_USB_Composite/Class/CDC_ACM/Inc
    ../common/AL94_USB_Composite/Class/COMPOSITE/Inc
    ../common/AL94_USB_Composite/Core/Inc
    ../common/AL94_USB_Composite/Target
    specific
)

file(GLOB_RECURSE VARMINT_SOURCES
    "Core/Src/*.c"
    "Core/Startup/*.s"
    "Drivers/*.c"
    "../common/*.cpp"
    "../common/drivers/*.cpp"
    "../common/AL94_USB_Composite/*.c"
    "specific/*.cpp"
)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32H7LinkerScript.ld)
add_link_options(-T ${LINKER_SCRIPT})

### preprocessor, compiler, linker options ###
# add_definitions(-DSTM32H753xx) was done in the CMakeLists.txt file that includes this one
add_definitions(-DDEBUG -DUSE_HAL_DRIVER)

add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-Wall)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${TARGET_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork -u _printf_float )

### build target ###

add_executable(${TARGET_NAME}.elf ${ROSFLIGHT_SOURCES} ${VARMINT_SOURCES} ${LINKER_SCRIPT})
target_compile_definitions(${TARGET_NAME}.elf PUBLIC
  GIT_VERSION_HASH=0x${GIT_VERSION_HASH}
  GIT_VERSION_STRING=\"${GIT_VERSION_STRING}\"
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.bin)

add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
