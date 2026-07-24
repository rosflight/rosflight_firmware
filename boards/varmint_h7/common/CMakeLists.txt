# This is not a top-level project. It is called from <board>/CMakeLists.txt
# test: see root CMakeLists.txt for magic words.

set(board_source_dir "${CMAKE_CURRENT_SOURCE_DIR}")
set(common_source_dir "${board_source_dir}/../common")

### source files ###
file(GLOB_RECURSE board_sources
  "${board_source_dir}/Core/Src/*.c"
  "${board_source_dir}/Core/Startup/*.s"
  "${board_source_dir}/Drivers/*.c"
  "${board_source_dir}/specific/*.cpp"
  "${common_source_dir}//*.cpp"
  "${common_source_dir}/drivers/*.cpp"
  "${common_source_dir}/AL94_USB_Composite/*.c"
)

### build target ###
add_executable(${TARGET_NAME}.elf ${board_sources})
target_compile_definitions(${TARGET_NAME}.elf PRIVATE DEBUG USE_HAL_DRIVER)
target_include_directories(${TARGET_NAME}.elf PRIVATE
  ${board_source_dir}/Core/Inc
  ${board_source_dir}/Drivers/STM32H7xx_HAL_Driver/Inc
  ${board_source_dir}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  ${board_source_dir}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
  ${board_source_dir}/Drivers/CMSIS/Include
  ${board_source_dir}/specific
  ${common_source_dir}
  ${common_source_dir}/drivers
  ${common_source_dir}/AL94_USB_Composite
)
target_link_libraries(${TARGET_NAME}.elf PRIVATE
  rosflight_firmware::rosflight_firmware
)

set(linker_script ${common_source_dir}/STM32H7LinkerScript.ld)
target_link_options(${TARGET_NAME}.elf PRIVATE
  "-T${linker_script}"
  "-Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.elf.map,--cref"
)
#${workspaceFolder}/build/${command:cmake.activeConfigurePresetName}/output/

set(hex_file ${PROJECT_BINARY_DIR}/${TARGET_NAME}.hex)
set(bin_file ${PROJECT_BINARY_DIR}/${TARGET_NAME}.bin)

add_custom_command(TARGET ${TARGET_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${TARGET_NAME}.elf> ${hex_file}
  COMMAND
    ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${TARGET_NAME}.elf> ${bin_file}
  COMMENT "Building Artifacts: ${hex_file} and ${bin_file}"
)
