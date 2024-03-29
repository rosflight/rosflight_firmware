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

echo_blue "Test 1: Build varmint firmware"
rm -rf build
mkdir build
cd build
cmake .. -DBUILD_VARMINT=TRUE -DCMAKE_BUILD_TYPE=RELEASE && make -j4 -l4
print_result $?
cd ..

echo_blue "Test 2: Build test suite"
rm -rf build
mkdir build
cd build
cmake .. -DBUILD_TEST=TRUE -DCMAKE_BUILD_TYPE=RELEASE && make -j4 -l4
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
