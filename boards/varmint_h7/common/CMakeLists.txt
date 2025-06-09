# test:
# cd pixracer_pro && mkdir build && cd build && cmake .. && make -j
# cd varmint_10X && mkdir build && cd build && cmake .. && make -j

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

set(CMAKE_C_FLAGS_DEBUG " ")
set(CMAKE_CXX_FLAGS_DEBUG " ")

### source files ###

# Add test directory if being compiled without rosflight_firmware. Normally ROSFLIGHT_SOURCES is
#  provided by the CMakeLists.txt file in the root of the firmware repo and has the source files
#  for the firmware.
if(NOT DEFINED ROSFLIGHT_SOURCES)
    include_directories(
		../test 
		../test/interface
	)
    set(ROSFLIGHT_SOURCES "../test/main.cpp")
endif()
	
if(NOT DEFINED TARGET_NAME)
    set(TARGET_NAME "unnamed")
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

### Pre-processor ###
# add_definitions(-DSTM32H753xx) was done in the CMakeLists.txt file that includes this one
add_definitions(-DDEBUG -DUSE_HAL_DRIVER)
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)

### Compiler and Assembler
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
# Debug settings
add_compile_options(-g3 -O3)
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

# Place global variables in BSS section of the object file (memory)
#add_compile_options(-fno-common)
# Do not wrap error messages
#add_compile_options( -fmessage-length=0)
# Use Thumb instructions ?
add_compile_options(-mthumb)
#add_compile_options(-mthumb-interwork) # can be removed, but used in pre v5 architectures
# Set MCU type
add_compile_options(-mcpu=cortex-m7)
# Compile but do not link
add_compile_options(-c)
# Place each function or data item into its own section in the output file (used to optimize link)
add_compile_options(-ffunction-sections -fdata-sections)

# Warnings
add_compile_options(-Wall)
add_compile_options(-fstack-usage)
#add_compile_options(-fcyclomatic-complexity)
# Floating point type
add_compile_options(-mfpu=fpv5-d16 -mfloat-abi=hard)
# System calls, Minimial implementation
add_compile_options(--specs=nosys.specs)


# c++ only options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-use-cxa-atexit")
# C-only options
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")

### Linker ###
# Set MCU type
add_link_options(-mcpu=cortex-m7)
# Use thumb instructions ?
add_link_options(-mthumb)
# Generate code that supports calling between ARM and Thumb instruction sets.
#add_link_options(-mthumb-interwork ) # can be removed, only used in pre v5 architectures
# Select linker script (memory allocation)
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32H7LinkerScript.ld)
add_link_options(-T ${LINKER_SCRIPT} )
# System calls, Minimial implementation
add_link_options(--specs=nosys.specs)
# Discared unused Sections
add_link_options( -Wl,--gc-sections,--print-memory-usage)
# Prefer static vs shared libraries
add_link_options( -static )
# Floating point type
add_link_options(-mfpu=fpv5-d16 -mfloat-abi=hard)
# Enable floating point printf
add_link_options(-u _printf_float)
# Use C math library
add_link_options( -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group )
# Create a map file
add_link_options( -Wl,-Map=${PROJECT_BINARY_DIR}/${TARGET_NAME}.map )
# Print out memory usage
add_link_options( -Wl,--print-memory-usage)





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
