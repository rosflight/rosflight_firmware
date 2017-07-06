# Building Firmware from source

You'll first need to install some 32-bit dependencies:

```bash
sudo apt-get install lib32ncurses5 lib32tinfo5 libc6-i386
```

Now let's download the compiler.  The easiest place to get these is from [launchpad](https://launchpad.net/gcc-arm-embedded/+download).  Download the latest `.tar.bz2` file for Linux, extract it, and put it somewere, like your `/opt` folder.

``` bash
cd ~/Downloads
tar -xvf gcc-arm-none-eabi-5_4-2016q3-20160926-linux.tar.bz2 
sudo mv gcc-arm-none-eabi-5_4-2016q3 /opt/.
```

For convenience, you might want to add the `bin` folder of the compiler to your path so you can call it from anywhere.  To do that, just append the following line to the bottom of your `~/.bashrc` file.

``` bash
export PATH=$PATH:/opt/gcc-arm-none-eabi-5_4-2016q3/bin
```

Now that we have the compiler installed, simply clone the firmware repository and build

``` bash
git clone https://github.com/rosflight/firmware
cd firmware
make
```

# Flashing Newly Built Firmware

Install the stm32flash utility

``` bash
git clone git://git.code.sf.net/p/stm32flash/code stm32flash-code
cd stm32flash-code
sudo make install
cd ..
rm -rf stm32flash-code
```

Then put the board in bootloader mode (short the boot pins while cycling power) and type `make flash`


# Building and Running Unit Tests

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


