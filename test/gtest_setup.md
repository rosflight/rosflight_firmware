# Setting up gtest for running unit tests

## Install gtest
``` bash
sudo apt-get install libgtest-dev
```

## Compile gtest
You just downloaded a bunch of source files, which you now have to go build

``` bash
sudo apt install cmake
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
```

Copy the archive files you just build to the `/usr/lib` directory so CMake can find them later

```
sudo cp *.a /usr/lib
```

## Build the Unit Tests

``` bash
cd <ROSflight_location>/test
mkdir build
cd build
cmake ..
make
```

## Run the Unit Tests
``` bash
./turbotrig_test
./state_machine_test
