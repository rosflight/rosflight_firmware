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

echo_blue "Test 1: Build F1 firmware"
make clean BOARD=NAZE
make BOARD=NAZE
print_result $?

echo_blue "Test 2: Build F4 firmware"
make clean BOARD=REVO
make BOARD=REVO
print_result $?

echo_blue "Test 3: Build test suite"
mkdir -p test/build
cd test/build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=RELEASE && make -j4 -l4
print_result $?

echo_blue "Test 4: Run test suite"
./unit_tests
print_result $?


if [ $EXIT_CODE -eq 0 ]; then
  echo_green "All tests passed!"
else
  echo_red "There were failed tests"
fi

exit $EXIT_CODE
