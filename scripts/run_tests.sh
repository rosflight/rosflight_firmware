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

rm -rf build
echo_blue "Test 1a: Build varmint_10X firmware"
cmake --preset varmint-10X-release
cmake --build build/varmint-10X-release
print_result $?

echo_blue "Test 1b: Build varmint_11X firmware"
cmake --preset varmint-11X-release
cmake --build build/varmint-11X-release
print_result $?


echo_blue "Test 1c: Build pixracer pro firmware"
cmake --preset pixracer-pro-release
cmake --build build/pixracer-pro-release
print_result $?

echo_blue "Test 2: Build test suite"

cmake --preset test-release
cmake --build build/test-release
print_result $?

echo_blue "Test 3: Run test suite"
./build/test-release/test/unit_tests
print_result $?

if [ $EXIT_CODE -eq 0 ]; then
  echo_green "All tests passed!"
else
  echo_red "There were failed tests"
fi

exit $EXIT_CODE
