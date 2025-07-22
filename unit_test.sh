#!/bin/bash

# Building the projects:
# mkdir varmint_build      && cd varmint_build      && cmake .. -DBOARD_TO_BUILD=varmint      && make -j
# mkdir pixracer_pro_build && cd pixracer_pro_build && cmake .. -DBOARD_TO_BUILD=pixracer_pro && make -j
#
# alternate for make -j above is cmake --build . -j (note only one .)
# make -j OR cmake --build . -j
# to clean:
# make clean -j  OR  make --build . --target clean -j
#
# unit test:
# build:
# mkdir test_build         && cd test_build         && cmake .. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j
# test:
# ./test/unit_tests

#!/bin/bash
#function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
#function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
#function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

#EXIT_CODE=0

#function print_result() {
#  if [ $1 -eq 0 ]; then
#    echo_green "[Passed]"
#  else
#    echo_red "[Failed]"
#    EXIT_CODE=1
#  fi
#  echo ""
#}

rm CMakeCache.txt

# Save the current directory
START_DIR=$(pwd)

# Create the build directory if it doesn't exist
mkdir -p build

# Empty build/test directory if it exists, otherwise create it
if [ -d "build/test" ]; then
    sudo rm -rf build/test/*
else
    mkdir -p build/test
fi

echo_blue "Test 1: Build test suite"
# Navigate to build/test directory
cd build/test

# Run cmake with specified parameters and build project in Release mode
cmake ../.. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j
#print_result $?

echo_blue "Test 2: Run test suite"
# Run unit tests
./test/unit_tests
#print_result $?

# Return to starting directory
cd "$START_DIR"



