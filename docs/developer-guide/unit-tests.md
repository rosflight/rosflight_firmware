# Building and Running Unit Tests

Contributions will need to pass our continuous integration unit tests before merging. To test your contributions against these tests, you'll first need to install Eigen and gtest:

``` bash
sudo apt install libgtest-dev libeigen3-dev cmake
```

## Compile gtest
You just downloaded a bunch of source files, which you now have to go build

``` bash
sudo apt install cmake
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
```

Copy the archive files you just built to the `/usr/lib` directory so CMake can find them later:

``` bash
sudo cp *.a /usr/lib
```

## Build the Unit Tests

Then you'll need to build and run the tests themselves:

``` bash
cd <firmware_directory>/test
mkdir build
cd build
cmake ..
make
```

## Run the Unit Tests

``` bash
./unit_tests
```
