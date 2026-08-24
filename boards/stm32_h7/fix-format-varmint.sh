#!/bin/bash

SCRIPT=$(readlink -f $0)
echo $SCRIPT
SCRIPTPATH=`dirname $SCRIPT`
echo $SCRIPTPATH
# cd $SCRIPTPATH/..

 find . -iname "*.h" -o -iname "*.c" -o -iname "*.cpp" \
    | grep -Ev "^*/../common/stm32_drivers|^*/Core" \
    | xargs clang-format -i --verbose -style=file
	
 #find . -iname "*.h" -o -iname "*.c" -o -iname "*.cpp" \
 #   | grep -Ev "^*/../common/stm32_drivers|^*/Core|^*/AL94_USB_Composite" \
 #   | xargs clang-format -i --verbose -style=file

#  find . -iname "*.h" -o -iname "*.c" -o -iname "*.cpp" \
#     | grep -Ev "^*/../common/stm32_drivers|^*/Core|^*/AL94_USB_Composite" \
#     | xargs clang-format --dry-run --Werror