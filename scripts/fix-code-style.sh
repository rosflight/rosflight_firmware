#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
echo #SCRIPTPATH
cd $SCRIPTPATH/..

find . -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \
| grep -Ev "^(./comms/mavlink/v1.0|./.git|./lib/eigen|./build|./CMakeFiles|.*/Core/|.*/Drivers/|.*/AL94_USB_Composite/)" \
| xargs clang-format -i --verbose -style=file