#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
# echo $SCRIPTPATH
cd $SCRIPTPATH/..

# format c/c++ code
find . \
  \( \
    -path "./.git" \
    -o -path "./comms/mavlink/v1.0" \
    -o -path "./lib/eigen" \
    -o -path "./build" \
    -o -path "*/Core" \
    -o -path "*/stm32_drivers" \
    -o -path "*/sensor_drivers" \
    -o -path "*/specific" \
    -o -path "*/AL94_USB_Composite" \
  \) -prune \
  -o \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) -print \
  | xargs clang-format -i --verbose -style=file

# organize python imports
ruff check --select I --fix ./scripts

# format python code
ruff format ./scripts
