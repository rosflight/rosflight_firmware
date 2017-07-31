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

echo_blue "Test 1: Build firmware"
make clean
make
print_result $?

echo_blue "Test 2: Build test suite"
mkdir -p test/build
cd test/build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=RELEASE && make
print_result $?

echo_blue "Test 3: Run test suite"
./unit_tests
print_result $?


if [ $EXIT_CODE -eq 0 ]; then
  echo_green "All tests passed!"
else
  echo_red "There were failed tests"
fi

exit $EXIT_CODE
