# Building and Flashing the Firmware

Additionally, make sure you have configured your computer as described in the [Serial Port Configuration](/user-guide/flight-controller-setup/#serial-port-configuration) in the User Guide.

!!! Warning
    Deprecation Notice: As of June 2019, plans are to deprecate support for the F1 in the near future. If you need to use an F1, you will need to retrieve an older version of the code that supports the F1. However, if there are issues, we will not be able to help you fix them.

## Building Firmware from Source

To build the firmware, you will need a supported version of the ARM embedded toolchain (the compiler). If you are using the Ubuntu operating on an ARM chip, you can simply install gcc with `apt`. Otherwise, you will need to use a different method to install the ARM gcc compiler. We recommend using the latest stable release provided on the [ARM](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) website. If you do use this method, we encourage you to verify the MD5 checksum after downloading. Currently (as of June 2019), we are targeting version 6.3.1 of the gcc-arm-none-eabi compiler provided by the [6-2017-q2-update](https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2?revision=2cc92fb5-3e0e-402d-9197-bdfc8224d8a5?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,6-2017-q2-update). The instructions for installing this version follow:

``` bash
cd Downloads
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2?revision=2cc92fb5-3e0e-402d-9197-bdfc8224d8a5?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,6-2017-q2-update
tar -xvf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
sudo mv gcc-arm-none-eabi-6-2017-q2-update /opt/.
echo "export PATH=$PATH:/opt/gcc-arm-none-eabi-6-2017-q2-update/bin" >> ~/.bashrc
rm -rf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
```

You can check which version installed by running `arm-none-eabi-gcc --version`.

Now that we have the compiler installed, simply clone the ROSflight firmware repository, pull down the submodules, and build:

``` bash
git clone https://github.com/rosflight/firmware
cd firmware
git submodule update --init --recursive
make
```

## Flashing Newly Built Firmware

### F4

* Flash the firmware to the board by running `make BOARD=REVO flash`

### F1

* Flash the firmware to the board by running `make BOARD=NAZE flash`

## Bonus

!!! Note
    You _**might**_ first need to install some 32-bit dependencies:
    ```bash
    sudo apt install lib32ncurses5 lib32tinfo5 libc6-i386
    ```

If you want to try to use the latest gcc compiler provided by ARM, use the following steps to install:

* Download the latest `.tar.bz2` file for Linux, [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* extract it, and put it somewhere, like your `/opt` folder (be sure to fill in the correct `<gcc_version>` with the one you downloaded)

``` bash
cd ~/Downloads
tar -xvf gcc-arm-none-eabi-<gcc_version>-linux.tar.bz2
sudo mv gcc-arm-none-eabi-<gcc_version> /opt/
```

For convenience, you might want to add the `bin` folder of the compiler to your path so you can call it from anywhere. To do that, just append the following line to the bottom of your `~/.bashrc` file:

``` bash
export PATH=$PATH:/opt/gcc-arm-none-eabi-<gcc_version>/bin
```
