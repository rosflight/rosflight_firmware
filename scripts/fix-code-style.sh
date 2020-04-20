#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
echo #SCRIPTPATH
cd $SCRIPTPATH/..

find . -iname "*.h" -o -iname "*.cpp" | \
egrep -v "^(./comms/mavlink/v1.0/|./boards/airbourne/airbourne/|./boards/breezy/breezystm32|./.git|./test/build)" | \
xargs clang-format -i
