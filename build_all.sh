#!/bin/bash

# Building the projects:
# mkdir varmint_build      && cd varmint_build      && cmake .. -DBOARD_TO_BUILD=varmint      && make -j
# mkdir pixracer_pro_build && cd pixracer_pro_build && cmake .. -DBOARD_TO_BUILD=pixracer_pro && make -j
#
# alternate for make -j above is cmake --build . -j (note only one .)
# make -j OR cmake --build . -j
# to clean:
# make clean -j  OR  make --build . --target clean -j
#
# unit test:
# build:
# mkdir test_build         && cd test_build         && cmake .. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j
# test:
# ./test/unit_tests

#!/bin/bash

# Save the current directory
START_DIR=$(pwd)

# Create the build directory if it doesn't exist
mkdir -p build

# Empty build/test directory if it exists, otherwise create it
if [ -d "build/test" ]; then
    sudo rm -rf build/test/*
else
    mkdir -p build/test
fi

# Navigate to build/test directory
cd build/test

# Run cmake with specified parameters and build project in Release mode
cmake ../.. -DBOARD_TO_BUILD=test -DCMAKE_BUILD_TYPE=Release && make -j

# Run unit tests
./test/unit_tests

# Return to starting directory
cd "$START_DIR"

# Build varmint project
# Empty build/varmint directory if it exists, otherwise create it
if [ -d "build/varmint" ]; then
    sudo rm -rf build/varmint/*
else
    mkdir -p build/varmint
fi

# Navigate to build/varmint directory
cd build/varmint

# Run cmake with specified parameters and build project
cmake ../.. -DBOARD_TO_BUILD=varmint && make -j

# Return to starting directory
cd "$START_DIR"

mv build/varmint/boards/varmint_h7/varmint/{*.elf,*.hex,*.bin} "$START_DIR/build/"


# Build pixracer project
# Empty build/pixracer_pro directory if it exists, otherwise create it
if [ -d "build/pixracer_pro" ]; then
    sudo rm -rf build/pixracer_pro/*
else
    mkdir -p build/pixracer_pro
fi

# Navigate to build/pixracer directory
cd build/pixracer_pro

# Run cmake with specified parameters and build project
cmake ../.. -DBOARD_TO_BUILD=pixracer_pro && make -j

# Return to starting directory
cd "$START_DIR"

mv build/pixracer_pro/boards/varmint_h7/pixracer_pro/{*.elf,*.hex,*.bin} "$START_DIR/build/"



