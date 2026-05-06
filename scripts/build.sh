#!/bin/bash

function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }
function echo_yellow { echo -e "\033[1;33m$@\033[0m"; }

EXIT_CODE=0

function print_result() {
  if [ $1 -eq 0 ]; then
    echo_green "[Passed]"
  else
    echo_red "[Failed]"
    EXIT_CODE=1
  fi
  echo ""
}

# Check if preset name is provided
if [ -z "$1" ]; then
   echo_red "Error: No preset name provided"
  echo_yellow "Usage: $0 <preset-name>"
  echo_yellow "Example: $0 varmint-10X-release"
  echo ""
  echo_yellow "Run 'cmake --list-presets' to see available presets, or"
  echo_yellow "run 'cmake --preset <invalid>' to see the full preset table"

  cmake --list-presets
  exit 1
fi

PRESET_NAME="$1"
ORIGINAL_DIR="$PWD"

# Change to project root if running from scripts directory
BASENAME=`basename "$PWD"`
if [ $BASENAME == "scripts" ]; then
  cd ..
fi

# Configure with CMake preset
echo_blue "Configuring CMake preset: $PRESET_NAME"
cmake --preset "$PRESET_NAME"
CONFIGURE_RESULT=$?
print_result $CONFIGURE_RESULT

# Only build if configure succeeded
if [ $CONFIGURE_RESULT -eq 0 ]; then
  echo_blue "Building preset: $PRESET_NAME"
  cmake --build "build/$PRESET_NAME"
  BUILD_RESULT=$?
  print_result $BUILD_RESULT
else
  echo_red "Configuration failed, skipping build"
  EXIT_CODE=1
fi

# Print final result
echo ""
if [ $EXIT_CODE -eq 0 ]; then
  echo_green "Build completed successfully!"
else
  echo_red "Build failed"
fi

cd "$ORIGINAL_DIR"
exit $EXIT_CODE
