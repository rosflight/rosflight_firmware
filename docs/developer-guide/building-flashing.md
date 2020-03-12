# Building and Flashing the Firmware

!!! warning "Deprecation Notice"
    As of June 2019, plans are to deprecate support for the F1 in the near future. If you need to use an F1, you will need to retrieve an older version of the code that supports the F1. However, if there are issues, we will not be able to help you fix them.

This guide assumes you are running Ubuntu 18.04, which is the currently supported development environment.

## Installing the ARM Embedded Toolchain

Currently (as of March 2020) we are targeting version 7.3.1 of the ARM embedded toolchain. This toolchain can be installed from the GNU Arm Embedded Toolchain PPA:

``` bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt install gcc-arm-embedded
```

You can test the installation and check which version is installed by running `arm-none-eabi-gcc --version`.

## Building the Firmware from Source

Now that we have the compiler installed, simply clone the ROSflight firmware repository, pull down the submodules, and build:

``` bash
git clone https://github.com/rosflight/firmware
cd firmware
git submodule update --init --recursive
make
```

To build only the F4 firmware, use `make BOARD=REVO`. To build only the F1 firmware, use `make BOARD=NAZE`.

## Flashing Newly-Built Firmware

First, make sure you have configured your computer as described in the [Serial Port Configuration](../user-guide/flight-controller-setup.md#serial-port-configuration) section of the user guide.

### F4

Flash the firmware to the board by running `make BOARD=REVO flash`.
If necessary, specify the serial port with `make BOARD=REVO SERIAL_DEVICE=/dev/ttyACM0 flash`.

### F1

Flash the firmware to the board by running `make BOARD=NAZE flash`
If necessary, specify the serial port with `make BOARD=REVO SERIAL_DEVICE=/dev/ttyUSB0 flash`.
