cmake_minimum_required(VERSION 3.8)

project(Pixhawk_6C_Mini C CXX ASM)

add_definitions(-DSTM32H743xx -DUSE_HAL_DRIVER)

set(PIXRACER_PRO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../pixracer_pro)
set(HAL_DRIVER_DIR ${PIXRACER_PRO_DIR}/Drivers/STM32H7xx_HAL_Driver)
set(CMSIS_DIR ${PIXRACER_PRO_DIR}/Drivers/CMSIS)
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../common)

file(GLOB PIXHAWK_6C_MINI_USB_SOURCES
    ${COMMON_DIR}/AL94_USB_Composite/*.c
)

set(PIXHAWK_6C_MINI_SOURCES
    Core/Src/main.c
    Core/Src/Pixhawk6CMiniBoard.cpp
    Core/Src/stm32h7xx_it.c
    ${PIXRACER_PRO_DIR}/Core/Src/system_stm32h7xx.c
    ${PIXRACER_PRO_DIR}/Core/Src/syscalls.c
    ${PIXRACER_PRO_DIR}/Core/Src/sysmem.c
    ${PIXRACER_PRO_DIR}/Core/Startup/startup_stm32h743iikx.s
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_cortex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_crc.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_crc_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_dma.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_flash.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_flash_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_gpio.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_pcd.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_pcd_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_pwr.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_pwr_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_rcc.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_rcc_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_sd.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_sd_ex.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_hal_spi.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_ll_sdmmc.c
    ${HAL_DRIVER_DIR}/Src/stm32h7xx_ll_usb.c
    ${PIXHAWK_6C_MINI_USB_SOURCES}
)

add_executable(${TARGET_NAME}.elf ${ROSFLIGHT_SOURCES} ${PIXHAWK_6C_MINI_SOURCES})

target_include_directories(${TARGET_NAME}.elf PRIVATE
    Core/Inc
    ${HAL_DRIVER_DIR}/Inc
    ${HAL_DRIVER_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
    ${CMSIS_DIR}/Include
    ${COMMON_DIR}
    ${COMMON_DIR}/drivers
    ${COMMON_DIR}/AL94_USB_Composite
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../comms/mavlink
)

target_compile_definitions(${TARGET_NAME}.elf PUBLIC
    GIT_VERSION_HASH=0x${GIT_VERSION_HASH}
    GIT_VERSION_STRING=\"${GIT_VERSION_STRING}\"
)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../common/STM32H7LinkerScript.ld)

target_link_options(${TARGET_NAME}.elf PRIVATE
    "-T${LINKER_SCRIPT}"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.elf.map,--cref"
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${TARGET_NAME}.bin)

add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}.elf> ${BIN_FILE}
    COMMENT "Building Artifacts: ${HEX_FILE} and ${BIN_FILE}"
)
