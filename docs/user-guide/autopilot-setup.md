# Autopilot setup

## Compatible hardware

As of march, 2017, ROSflight is only supported on flight controllers with STM32F103 processors.  Specifically, the naze32 and the flip32.  Both the 6-DOF and 10-DOF versions of each board are fully supported.  We have had most success with flip32 boards purchased directly from [readytoflyquads.com](www.readytoflyquads.com).  We have had weird issues with knock-off boards from chinese vendors.

## Flashing firmware using the Cleanflight Configurator

1. download the latest version of ROSflight [here](https://github.com/byu-magicc/ROSflight/releases).
2. Install the cleanflight configurator - It is an extension to Google Chrome - [link](https://chrome.google.com/webstore/detail/cleanflight-configurator/enacoimjcgeinfnnnpajinjgmkahmfgb?hl=en)
3. Be sure your user is in the dialout group so you have access to the serial ports
4. Load the firmware and flash using cleanflight configurator

    * Open the configurator, open firmware flasher.  Connect your flight controller, and make sure that you have selected the right port.  Then select "Load Firmware (Local)" and  select your .hex file you downloaded earlier.
![cleanflight_gui_1](images/cleanflight_configurator-1.png)
    * Short the boot pins on your flight controller, unplug it, and and plug back in.
![boot_pins](images/boot_pins.png)
* If you scroll to the bottom of the screen, you will then see the green bar indicate progress as the hex is flashed to the flight controller.  You should then see something like the following 
![success](images/sucessful_flash.png)


## Flashing Firmware using stm32flash (CLI)

You can also use stm32flash to flash firmware.  This is helpful if you need a command-line interface.

1. Download and install stm32flash
``` bash
git clone git://git.code.sf.net/p/stm32flash/code stm32flash-code
cd stm32flash-code
sudo make install
cd ..
rm -rf stm32flash-code
```
2. Make sure you are in the dialout group (same instructions as above)
3. Short boot pins, restart the naze (by unplugging and plugging back in)
4. Download the latest ROSflight.hex file [here](https://github.com/byu-magicc/ROSflight/releases).
5. Flash the firmware to the proper device (replace `/dev/ttyUSB0`)
``` bash
    stm32flash -w ROSflight.hex -v -g 0x0 -b 921600 /dev/ttyUSB0
```




    
