# Building and Flashing the Firmware

You _**might**_ first need to install some 32-bit dependencies:

```bash
sudo apt install lib32ncurses5 lib32tinfo5 libc6-i386
```

## Building Firmware from Source

To build the firmware, you will need a supported version of the ARM embedded toolchain (the compiler). If you are using the Ubuntu operating on an ARM chip, you can simply install gcc with `apt`. Otherwise, you will need to use a different method to install the ARM gcc compiler. You may attempt to use the latest stable release provided on the [ARM](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) website, but we cannot guarantee an error-free experience. If you do use that method, we encourage you to verify the MD5 checksum. Currently, we are targeting version 6.3.1 of the gcc-arm-none-eabi compiler provided by the team-gcc-arm-embedded Ubuntu ppa. The instructions for installing this version follow, with the first command lessening the likelihood of an error while adding the ppa to your system:

``` bash
sudo apt install --reinstall ca-certificates
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt update
sudo apt install gcc-arm-none-eabi
```

You can check which version installed by running `arm-none-eabi-gcc --version`.

Now that we have the compiler installed, simply clone the ROSflight firmware repository, pull down the submodules, and build:

``` bash
git clone https://github.com/rosflight/firmware
cd firmware
git submodule update --init --recursive
make
```

## Flashing Newly Built Firmware (F4 & F1)

See [Autopilot Setup](/user-guide/autopilot-setup) for full instructions.

## Bonus

If you want to try to use the latest gcc compiler provided by ARM, use the following steps to install:

* Download the latest `.tar.bz2` file for Linux, [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
* extract it, and put it somewhere, like your `/opt` folder (be sure to fill in the correct `<gcc_version>` with the one you downloaded)

``` bash
cd ~/Downloads
tar -xvf gcc-arm-none-eabi-<gcc_version>-linux.tar.bz2
sudo mv gcc-arm-none-eabi-5_4-2016q3 /opt/
```

For convenience, you might want to add the `bin` folder of the compiler to your path so you can call it from anywhere. To do that, just append the following line to the bottom of your `~/.bashrc` file:

``` bash
export PATH=$PATH:/opt/gcc-arm-none-eabi-<gcc_version>/bin
```
