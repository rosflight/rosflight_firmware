# test: see root CMakeLists.txt for magic words.

### project settings ###

cmake_minimum_required(VERSION 3.8)

if(NOT DEFINED TARGET_NAME)
    set(TARGET_NAME "varmint_h7")
endif()

project(${TARGET_NAME} C CXX ASM)

### Pre-processor ###
# Add chip-specific definition based on BOARD_CHIP
if(DEFINED BOARD_CHIP)
	add_definitions(-DSTM32${BOARD_CHIP}xx)
else()
	message(FATAL_ERROR "BOARD_CHIP not defined. Set it in board-specific CMakeLists.txt")
endif()

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
	
# Common STM32 HAL drivers (consolidated)
file(GLOB_RECURSE COMMON_HAL_SOURCES
	"../common/stm32_drivers/STM32H7xx_HAL_Driver/Src/*.c"
)

# Common system files (consolidated)
set(COMMON_SYSTEM_SOURCES
	"../common/Core/Src/syscalls.c"
	"../common/Core/Src/sysmem.c"
	"../common/Core/Src/system_stm32h7xx.c"
)

# Board-specific startup file (chip-dependent)
if(DEFINED BOARD_CHIP)
	if(BOARD_CHIP STREQUAL "H753")
		set(STARTUP_FILE "../common/Core/Startup/startup_stm32h753vihx.s")
	elseif(BOARD_CHIP STREQUAL "H743")
		set(STARTUP_FILE "../common/Core/Startup/startup_stm32h743iikx.s")
	else()
		message(FATAL_ERROR "Unknown BOARD_CHIP: ${BOARD_CHIP}. Must be H753 or H743")
	endif()
else()
	# Default to H753 if not specified (for backward compatibility)
	set(STARTUP_FILE "../common/Core/Startup/startup_stm32h753vihx.s")
	message(WARNING "BOARD_CHIP not defined, defaulting to H753 startup file")
endif()

# Board-specific sources (excluding system files now in common)
file(GLOB_RECURSE VARMINT_SOURCES_ALL
	"Core/Src/*.c"
	"../common/*.cpp"
	"../common/sensor_drivers/*.cpp"
	"../common/AL94_USB_Composite/*.c"
	"specific/*.cpp"
)

# Exclude syscalls.c and sysmem.c from board-specific sources since they're now in common
list(FILTER VARMINT_SOURCES_ALL EXCLUDE REGEX ".*/syscalls\\.c$")
list(FILTER VARMINT_SOURCES_ALL EXCLUDE REGEX ".*/sysmem\\.c$")
set(VARMINT_SOURCES ${VARMINT_SOURCES_ALL})

# Combine all sources
set(ALL_SOURCES
	${COMMON_HAL_SOURCES}
	${COMMON_SYSTEM_SOURCES}
	${STARTUP_FILE}
	${VARMINT_SOURCES}
)

include_directories(
	Core/Inc
	../common/stm32_drivers/STM32H7xx_HAL_Driver/Inc
	../common/stm32_drivers/STM32H7xx_HAL_Driver/Inc/Legacy
	../common/stm32_drivers/CMSIS/Device/ST/STM32H7xx/Include
	../common/stm32_drivers/CMSIS/Include
	../common
	../common/sensor_drivers
	../common/AL94_USB_Composite
	specific
)

### build target ###
add_executable(${TARGET_NAME}.elf ${ROSFLIGHT_SOURCES} ${ALL_SOURCES})
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32H7LinkerScript.ld)

target_link_options(${TARGET_NAME}.elf PRIVATE
    "-T${LINKER_SCRIPT}"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.elf.map,--cref"
)
#${workspaceFolder}/build/${command:cmake.activeConfigurePresetName}/output/

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