# run as admin from the varmint_h7 folder
cd ..\..\..
mkdir links
cd links
mklink /D comms   ..\rosflight_firmware\comms
mklink /D include ..\rosflight_firmware\include
mklink /D lib     ..\rosflight_firmware\lib
mklink /D src     ..\rosflight_firmware\src
mklink /D common  ..\rosflight_firmware\boards\varmint_h7\common
cd ..\rosflight_firmware\boards\varmint_h7
