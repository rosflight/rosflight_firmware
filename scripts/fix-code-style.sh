#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
# echo $SCRIPTPATH
cd $SCRIPTPATH/..

find . -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \
| grep -Ev "^(./comms/mavlink/v1.0|./.git|./lib/eigen|./build|./CMakeFiles|.*/boards/)" \
| xargs clang-format -i --verbose -style=file
