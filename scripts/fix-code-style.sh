#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
# echo $SCRIPTPATH
cd $SCRIPTPATH/..

# format c/c++ code
find . \
  \( \ # exclude list
    -path "./.git" \
    -o -path "./comms/mavlink/v1.0" \
    -o -path "./lib/eigen" \
    -o -path "./build" \
    -o -path "./CMakeFiles" \
    -o -path "*/Core" \
    -o -path "*/Drivers" \
    -o -path "*/drivers" \
    -o -path "*/specific" \
    -o -path "*/AL94_USB_Composite" \
  \) -prune \
  -o \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) -print \
  | xargs clang-format -i --verbose -style=file

# organize python imports
find ./scripts -iname "*.py" | xargs -r ruff check --select I --fix

# format python code
find ./scripts -iname "*.py" | xargs -r ruff format
