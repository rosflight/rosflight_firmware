# test:
# cd pixracer_pro && mkdir build && cd build && cmake .. && make -j
# cd varmint_10X && mkdir build && cd build && cmake .. && make -j

### project settings ###

cmake_minimum_required(VERSION 3.8)

if(NOT DEFINED TARGET_NAME)
    set(TARGET_NAME "varmint_h7")
endif()

project(${TARGET_NAME} C CXX ASM)

### Pre-processor ###
add_definitions(-DDEBUG -DUSE_HAL_DRIVER)

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
	
file(GLOB_RECURSE VARMINT_SOURCES
	"Core/Src/*.c"
	"Core/Startup/*.s"
	"Drivers/*.c"
	"../common/*.cpp"
	"../common/drivers/*.cpp"
	"../common/AL94_USB_Composite/*.c"
	"specific/*.cpp"
)

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

### build target ###
add_executable(${TARGET_NAME}.elf ${ROSFLIGHT_SOURCES} ${VARMINT_SOURCES})
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32H7LinkerScript.ld)

target_link_options(${TARGET_NAME}.elf PRIVATE
    "-T${LINKER_SCRIPT}"
    "-Wl,-Map=${PROJECT_BINARY_DIR}/${TARGET_NAME}.map"
)

target_compile_definitions(${TARGET_NAME}.elf PUBLIC
    GIT_VERSION_HASH=0x${GIT_VERSION_HASH}
    GIT_VERSION_STRING=\"${GIT_VERSION_STRING}\"
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.bin)

add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}.elf> ${BIN_FILE}
	COMMENT "Building Artifacts: ${HEX_FILE} and ${BIN_FILE}"
)