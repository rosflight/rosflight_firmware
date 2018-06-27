#################################
# GNU ARM Embedded Toolchain
#################################
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line

#################################
# DIRECTORIES
#################################
ROSFLIGHT_DIR   = ../..
TURBOMATH_DIR   = $(ROSFLIGHT_DIR)/lib/turbomath
MAVLINK_DIR     = $(ROSFLIGHT_DIR)/comms/mavlink

#################################
# SOURCE FILES
#################################

# MAVLink source files
VPATH := $(VPATH):$(MAVLINK_DIR)
MAVLINK_SRC = mavlink.cpp

# ROSflight source files
VPATH		:= $(VPATH):$(ROSFLIGHT_DIR):$(ROSFLIGHT_DIR)/src
ROSFLIGHT_SRC = rosflight.cpp \
                param.cpp \
                sensors.cpp \
                state_manager.cpp \
                estimator.cpp \
                controller.cpp \
                comm_manager.cpp \
                command_manager.cpp \
                rc.cpp \
                mixer.cpp \
                nanoprintf.cpp

# Math Source Files
VPATH := $(VPATH):$(TURBOMATH_DIR)
MATH_SRC =  turbomath.cpp

# List of Common C++ Source Files
CXXSOURCES = $(ROSFLIGHT_SRC) \
             $(MATH_SRC) \
             $(MAVLINK_SRC)

# No C sources in common
CSOURCES =

# Common Include Files
INCLUDE_DIRS =  $(ROSFLIGHT_DIR)/include \
                $(ROSFLIGHT_DIR)/lib \
                $(MAVLINK_DIR)

#################################
# BUILD TYPE
#################################
ifeq ($(DEBUG), GDB)
DEBUG_FLAGS = -ggdb
OPTIMIZE = -O0
$(info ***** Building with Debug Symbols *****)
BUILD_TYPE=Debug
else
DEBUG_FLAGS = -ggdb
OPTIMIZE = -O2
COMPILE_FLAGS = -flto
LTO_FLAGS = -flto -fuse-linker-plugin $(OPTIMIZE)
BUILD_TYPE=Release
endif


#################################
# VERSION CONTROL
#################################
GIT_VERSION_HASH := $(shell git rev-parse --short=8 HEAD)
GIT_VERSION_STRING := $(shell git describe --tags --abbrev=8 --always --dirty --long)
GIT_VARS := -DGIT_VERSION_HASH=0x$(GIT_VERSION_HASH) -DGIT_VERSION_STRING=\"$(GIT_VERSION_STRING)\"

#################################
# Object List
#################################
OBJECTS=$(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(ASOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(CSOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(BIN_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET)_$(BOARD)_$(BUILD_TYPE).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET)_$(BOARD)_$(BUILD_TYPE).hex
TARGET_BIN=$(BIN_DIR)/$(TARGET)_$(BOARD)_$(BUILD_TYPE).bin

#################################
# Common Flags
#################################
DEFS = -DTARGET_$(BOARD) $(GIT_VARS)
CXX_STRICT_FLAGS += -pedantic -pedantic-errors -Werror -Wall -Wextra \
  -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 -Wlogical-op -Wmissing-include-dirs \
  -Wredundant-decls -Wshadow -Wstrict-overflow=2 -Wswitch -Wundef -Wunused -Wvariadic-macros \
  -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel
FILE_SIZE_FLAGS += -ffunction-sections -fdata-sections -fno-exceptions
CXX_FILE_SIZE_FLAGS =-c $(FILE_SIZE_FLAGS) -fno-rtti
CFLAGS   = -c $(DEFS) $(DEBUG_FLAGS) $(FILE_SIZE_FLAGS) -std=c99
CXXFLAGS = -c $(DEFS) $(DEBUG_FLAGS) $(CXX_FILE_SIZE_FLAGS) $(CXX_STRICT_FLAGS) -std=c++11
