# Developer Documentation

These documents are designed to help developers get up and running with developing new features and understanding the internals of the firmware.  As of version 0.1, development is only supported on Ubuntu Linux 16.04.

## Building Firmware from source

To build the firmware, you will need the latest version of the ARM embedded toolchain.  This can be done with the following commands:

``` bash
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt update
sudo apt install lib32ncurses5 lib32tinfo5 libc6-i386 gcc-arm-embedded
```

Then, simply clone the repository and build

``` bash
git clone https://github.com/rosflight/firmware
cd firmware
make
```

## Flashing Newly Built Firmware

Install the stm32flash utility

``` bash
git clone git://git.code.sf.net/p/stm32flash/code stm32flash-code
cd stm32flash-code
sudo make install
cd ..
rm -rf stm32flash-code
```

Then put the board in bootloader mode (short the boot pins while cycling power) and type `make flash`


## Building and Running Unit Tests

Contributions will need to pass our continuous integration unit tests before merging.  To test your contributions against these tests, you'll first need to install Eigen and gtest

``` bash
sudo apt install libgtest-dev libeigen3-dev cmake
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
```

Then you'll need to build and run the tests themselves

``` bash
cd <firmware_directory>/test
mkdir build
cd build
cmake ..
make
./unit_tests
```


