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

## Run the Test Script

The simplest way to run the unit tests is to use the testing script.
This script first checks that the firmware compiles, then runs the unit tests.
This is the same script used on the continuous integration server, so this is a great way to check that your code will pass the tests before opening a pull request.
Run the test script with

```bash
cd <firmware_directory>
./scripts/run_tests.sh
```

## Manually Build and Run the Unit Tests

If you want to manually build and run the unit tests, first build them with the following commands:

``` bash
cd <firmware_directory>/test
mkdir build
cd build
cmake ..
make
```

Then run them with:

``` bash
./unit_tests
```
