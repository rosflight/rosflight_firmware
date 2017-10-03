# Building and Flashing the firmware

These documents are designed to help developers get up and running with developing new features and understanding the internals of the firmware.  Development is currently supported only on Ubuntu Linux 16.04.

## Building firmware from source

To build the firmware, you will need the latest version of the ARM embedded toolchain.  We have had issues with compatability between versions of the gcc compiler.  Sometimes, the latest version works, and other times it does not.  To be safe, install version `5_4-2016q3`.  It appears that version 6 is not working.  The following commands will install the 32-bit dependencies, download the compiler and install it to the /opt/ directory.  It also adds it to the `PATH`

``` bash
sudo apt install -y lib32ncurses5
wget https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q3-update/+download/gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2
tar -xvf gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2
sudo mv gcc-arm-none-eabi-5_4-2016q3 /opt/.
echo "export PATH=\$PATH:/opt/gcc-arm-none-eabi-5_4-2016q3/bin" >> ~/.bashrc
rm -rf gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2

```

Then, simply clone the repository, pull down the submodules, and build:

``` bash
git clone https://github.com/rosflight/firmware
cd firmware
git submodule update --init --recursive
make
```

## Flashing newly built firmware

Install the stm32flash utility

``` bash
git clone git://git.code.sf.net/p/stm32flash/code stm32flash-code
cd stm32flash-code
sudo make install
cd ..
rm -rf stm32flash-code
```

Then put the board in bootloader mode (short the boot pins while cycling power) and type `make flash`


## Building and running unit tests

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
