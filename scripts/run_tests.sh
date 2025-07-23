#!/bin/bash

function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

EXIT_CODE=0

function print_result() {
  if [ $1 -eq 0 ]; then
    echo_green "[Passed]"
  else
    echo_red "[Failed]"
    EXIT_CODE=1
  fi
  echo ""
}

BASENAME=`basename "$PWD"`

if [ $BASENAME == "scripts" ]; then
  cd ..
fi

echo_blue "Test 1a: Build varmint_10X firmware"
rm -rf build
mkdir build
cd build
cmake .. -DBOARD_TO_BUILD=varmint_10X -DCMAKE_BUILD_TYPE=Release && make -j
print_result $?
cd ..

echo_blue "Test 1b: Build varmint_11X firmware"
rm -rf build
mkdir build
cd build
cmake .. -DBOARD_TO_BUILD=varmint_11X -DCMAKE_BUILD_TYPE=Release && make -j
print_result $?
cd ..

echo_blue "Test 1: Build pixracer pro firmware"
rm -rf build
mkdir build
cd build
cmake .. -DBOARD_TO_BUILD=pixracer_pro -DCMAKE_BUILD_TYPE=Release && make -j
print_result $?
cd ..

echo_blue "Test 2: Build test suite"
rm -rf build
mkdir build
cd build
cmake .. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j
print_result $?

echo_blue "Test 3: Run test suite"
./test/unit_tests
print_result $?


if [ $EXIT_CODE -eq 0 ]; then
  echo_green "All tests passed!"
else
  echo_red "There were failed tests"
fi

exit $EXIT_CODE
