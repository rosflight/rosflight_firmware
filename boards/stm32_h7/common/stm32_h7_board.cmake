function(add_stm32_h7_board board_name board_chip)
  # This function is expected to be called from the board's source directory.
  # If it ever is called from somewhere else then board_source_dir may need to
  # be passed into the function.
  set(board_source_dir "${CMAKE_CURRENT_SOURCE_DIR}")
  set(common_source_dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
  set(supported_board_chips H753 H743)

  ###############
  ## Prechecks ##
  ###############

  if(NOT TARGET rosflight_firmware::firmware_main)
    message(FATAL_ERROR "add_stm32_h7_board() "
      "requires rosflight_firmware::firmware_main to be defined first"
    )
  endif()

  if(NOT board_chip IN_LIST supported_board_chips)
    list(JOIN supported_board_chips ", " supported_board_chips_message)
    message(FATAL_ERROR "Unknown board_chip: ${board_chip}.\n"
      "Valid options: ${supported_board_chips_message}"
    )
  endif()

  ##################
  ## Source Files ##
  ##################

  # Common STM32 HAL drivers (consolidated)
  file(GLOB_RECURSE common_hal_sources CONFIGURE_DEPENDS
    "${common_source_dir}/stm32_drivers/STM32H7xx_HAL_Driver/Src/*.c"
  )

  # Common system files (consolidated)
  set(common_system_sources
    "${common_source_dir}/Core/Src/syscalls.c"
    "${common_source_dir}/Core/Src/sysmem.c"
    "${common_source_dir}/Core/Src/system_stm32h7xx.c"
  )

  # Board-specific startup file (chip-dependent)
  if(board_chip STREQUAL "H753")
    set(startup_file
      "${board_source_dir}/Core/Startup/startup_stm32h753vihx.s"
    )
  else()
    set(startup_file
      "${board_source_dir}/Core/Startup/startup_stm32h743iikx.s"
    )
  endif()

  # Board-specific sources (excluding system files now in common)
  file(GLOB_RECURSE board_sources CONFIGURE_DEPENDS
    "${board_source_dir}/Core/Src/*.c"
    "${board_source_dir}/specific/*.cpp"
    "${common_source_dir}/*.cpp"
    "${common_source_dir}/sensor_drivers/*.cpp"
    "${common_source_dir}/AL94_USB_Composite/*.c"
  )
  # Exclude syscalls.c and sysmem.c since they're now in common
  list(FILTER board_sources EXCLUDE REGEX ".*/syscalls\\.c$")
  list(FILTER board_sources EXCLUDE REGEX ".*/sysmem\\.c$")

  #####################
  ## Main Executable ##
  #####################

  set(target_name "${board_name}.elf")

  add_executable("${target_name}")
  target_sources("${target_name}" PRIVATE
    ${common_hal_sources}
    ${common_system_sources}
    "${startup_file}"
    ${board_sources}
  )
  target_compile_definitions("${target_name}" PRIVATE
    DEBUG
    USE_HAL_DRIVER
    "STM32${board_chip}xx"
  )
  target_compile_features("${target_name}" PRIVATE c_std_11 cxx_std_17)
  target_include_directories("${target_name}" PRIVATE
    "${board_source_dir}/Core/Inc"
    "${board_source_dir}/specific"
    "${common_source_dir}"
    "${common_source_dir}/stm32_drivers/STM32H7xx_HAL_Driver/Inc"
    "${common_source_dir}/stm32_drivers/STM32H7xx_HAL_Driver/Inc/Legacy"
    "${common_source_dir}/stm32_drivers/CMSIS/Device/ST/STM32H7xx/Include"
    "${common_source_dir}/stm32_drivers/CMSIS/Include"
    "${common_source_dir}/sensor_drivers"
    "${common_source_dir}/AL94_USB_Composite"
  )
  target_link_libraries("${target_name}" PRIVATE
    rosflight_firmware::firmware_main
  )

  set(linker_script "${common_source_dir}/STM32H7LinkerScript.ld")
  target_link_options("${target_name}" PRIVATE
    "-T${linker_script}"
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${board_name}.elf.map,--cref"
  )

  set(hex_file "${CMAKE_CURRENT_BINARY_DIR}/${board_name}.hex")
  set(bin_file "${CMAKE_CURRENT_BINARY_DIR}/${board_name}.bin")

  add_custom_command(TARGET "${target_name}" POST_BUILD
    BYPRODUCTS
      "${hex_file}"
      "${bin_file}"
    COMMAND
      "${CMAKE_OBJCOPY}"
      -O ihex
      "$<TARGET_FILE:${target_name}>"
      "${hex_file}"
    COMMAND
      "${CMAKE_OBJCOPY}"
      -O binary
      "$<TARGET_FILE:${target_name}>"
      "${bin_file}"
    COMMENT "Building Artifacts: ${hex_file} and ${bin_file}"
    VERBATIM
  )
endfunction()
