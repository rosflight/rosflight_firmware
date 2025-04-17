clear

# Check if the directory argument is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <varmint|pixracer_pro|test>"
  exit 1
fi

# Save the current directory
current_dir=$(pwd)
mkdir -p build/"$1"

# Change to the specified directory
cd build/"$1" || { echo "Failed to change directory to build/$1"; exit 1; }

if [ "$1" = "test" ]; then
  cmake ../.. -DBOARD_TO_BUILD="$1" -DCMAKE_BUILD_TYPE=Release
else
  cmake ../.. -DBOARD_TO_BUILD="$1"
fi

make -j
#cmake --build . -j

# Run unit tests if the argument is 'test'
if [ "$1" = "test" ]; then
  ./test/unit_tests
fi

# Return to the original directory
cd "$current_dir"

# Delete the build directory
rm -rf build/"$1"





