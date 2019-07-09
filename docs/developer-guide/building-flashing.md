# Building and Flashing the Firmware

!!! Warning
    Deprecation Notice: As of June 2019, plans are to deprecate support for the F1 in the near future. If you need to use an F1, you will need to retrieve an older version of the code that supports the F1. However, if there are issues, we will not be able to help you fix them.

## Installing the ARM Embedded Toolchain

To build the firmware, you will need a supported version of the ARM embedded toolchain (the compiler). If you are running Ubuntu on an ARM computer, you can simply install gcc with `apt`. Otherwise, you will need to manually install the ARM gcc compiler.

Currently (as of June 2019), we are targeting version 6.3.1 of the "gcc-arm-none-eabi" compiler provided by the [6-2017-q2-update](https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2).
Install this version of the toolchain by downloading the archive from the ARM website and extracting to your `/opt` directory:

```bash
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
tar -C /opt -xf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
```

Add the following line to `~/.bashrc` (or your equivalent) to add the toolchain to your path:

```bash
export PATH=$PATH:/opt/gcc-arm-none-eabi-6-2017-q2-update/bin
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
