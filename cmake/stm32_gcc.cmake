set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain definitions
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE_UTIL ${TOOLCHAIN_PREFIX}size)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)

set(H7_HW_FLAGS 
    -mcpu=cortex-m7 
    -mthumb 
    -mfpu=fpv5-d16 
    -mfloat-abi=hard 
    --specs=nosys.specs
)


# Architecture Specific Flags (The "Hardware" description)
add_compile_options(
    "${H7_HW_FLAGS}"
 #   -Wall                # All warnings
 #   -g3                  # Max debug symbols for J-Link (find in CMakePresets.json)
 #   -O3                  # High optimization for flight logic (find in CMakePresets.json)
    # -fstack-usage        # Generate .su files for RAM auditing
    # -ffunction-sections  # For linker garbage collection
    # -fdata-sections      # For linker garbage collection
    # Hardware Facts
    # -mcpu=cortex-m7 
    # -mthumb 
    # -mfpu=fpv5-d16 
    # -mfloat-abi=hard
    # Code Quality and Safety
    -Wall
    -ffunction-sections 
    -fdata-sections
    -fstack-usage
    # -fcyclomatic-complexity
    # System Integration
    --specs=nano.specs
    # Logic for STM32Cube's C-style comments in .s files
    "$<$<COMPILE_LANGUAGE:ASM>:-xassembler-with-cpp>"
    # C++ Specific: Disable bloat for embedded
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>"
)

add_link_options(
    "${H7_HW_FLAGS}"
#   "-T${LINKER_SCRIPT}" # should be here somehow, but moved to board/varmint_h7/common/CMakeLists.txt
#   "-Wl,-Map=${PROJECT_BINARY_DIR}/output/${TARGET_NAME}.map" # should be here somehow, but moved to board/varmint_h7/common/CMakeLists.txt
    -static
    "-Wl,--gc-sections"          
    -Wl,--print-memory-usage
    -u _printf_float
    -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
)

# Prevent CMake from failing the compiler test
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

