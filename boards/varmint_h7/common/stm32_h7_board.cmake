function(add_stm32_h7_board board_name mcu_definition)
  # This function is expected to be called from the board's source directory.
  # If it ever is called from somewhere else then board_source_dir may need to
  # be passed into the function.
  set(board_source_dir "${CMAKE_CURRENT_SOURCE_DIR}")
  set(common_source_dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")

  if(NOT TARGET rosflight_firmware::firmware_main)
    message(FATAL_ERROR "add_stm32_h7_board() "
      "requires rosflight_firmware::firmware_main to be defined first"
    )
  endif()

  ### source files ###
  file(GLOB_RECURSE board_sources CONFIGURE_DEPENDS
    "${board_source_dir}/Core/Src/*.c"
    "${board_source_dir}/Core/Startup/*.s"
    "${board_source_dir}/Drivers/*.c"
    "${board_source_dir}/specific/*.cpp"
    "${common_source_dir}/*.cpp"
    "${common_source_dir}/drivers/*.cpp"
    "${common_source_dir}/AL94_USB_Composite/*.c"
  )

  ### build target ###
  add_executable(${board_name}.elf ${board_sources})
  target_compile_definitions(${board_name}.elf PRIVATE
    DEBUG
    USE_HAL_DRIVER
    ${mcu_definition}
  )
  target_compile_features(${board_name}.elf PRIVATE c_std_11 cxx_std_17)
  target_include_directories(${board_name}.elf PRIVATE
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
  target_link_libraries(${board_name}.elf PRIVATE
    rosflight_firmware::firmware_main
  )

  set(linker_script "${common_source_dir}/STM32H7LinkerScript.ld")
  target_link_options(${board_name}.elf PRIVATE
    "-T${linker_script}"
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${board_name}.elf.map,--cref"
  )

  set(hex_file "${CMAKE_CURRENT_BINARY_DIR}/${board_name}.hex")
  set(bin_file "${CMAKE_CURRENT_BINARY_DIR}/${board_name}.bin")

  add_custom_command(TARGET ${board_name}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${board_name}.elf> ${hex_file}
    COMMAND
      ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${board_name}.elf> ${bin_file}
    COMMENT "Building Artifacts: ${hex_file} and ${bin_file}"
  )
endfunction()

