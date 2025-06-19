cmake_minimum_required(VERSION 3.8)
project(rosflight_tests)

add_definitions(-DGIT_VERSION_HASH=0x${GIT_VERSION_HASH})
add_definitions(-DGIT_VERSION_STRING=\"${GIT_VERSION_STRING}\")

# Locate GTest
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)  # Required for current gtest version, may be a bug with gtest?
include_directories(${GTEST_INCLUDE_DIRS})

# Compile eigen as a separate library and suppress warnings for eigen
add_library(eigen INTERFACE ../lib/eigen)
target_compile_options(eigen INTERFACE -Wno-strict-overflow -Wno-old-style-cast -Wno-ctor-dtor-privacy)

add_executable(unit_tests
        ${ROSFLIGHT_SOURCES}
        common.cpp
        command_manager_test.cpp
        test_board.cpp
        turbotrig_test.cpp
        state_machine_test.cpp
        command_manager_test.cpp
        estimator_test.cpp
        parameters_test.cpp
        rosflight_test.cpp
        )

target_include_directories(unit_tests PUBLIC .)
target_link_libraries(unit_tests ${GTEST_LIBRARIES} pthread eigen)

# Strict Compilation
target_compile_options(unit_tests PRIVATE -pedantic -pedantic-errors -Werror -Wall -Wextra 
  -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 -Wlogical-op -Wmissing-include-dirs 
  -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wundef -Wunused -Wvariadic-macros 
  -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel 
  ${CMAKE_CXX_FLAGS})

target_compile_definitions(unit_tests PRIVATE BUILD_TEST_BOARD)
