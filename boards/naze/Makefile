###############################################################################
# Generic Makefile Template for C/C++ for use with STM32 Microcontrollers
#
# Copyright (c) 2016 - James Jackson
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TARGET	?= rosflight

DEBUG ?= 

SERIAL_DEVICE ?= /dev/ttyUSB0

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
# Working directories
#################################
ROOT		= $(dir $(lastword $(MAKEFILE_LIST)))
BOARD_DIR 	= .
ROSFLIGHT_DIR   = ../..
MAVLINK_DIR = $(ROSFLIGHT_DIR)/comms/mavlink
TURBOMATH_DIR   = $(ROSFLIGHT_DIR)/lib/turbomath
BREEZY_DIR	= lib/breezystm32
CMSIS_DIR	= $(BREEZY_DIR)/lib/CMSIS
STDPERIPH_DIR	= $(BREEZY_DIR)/lib/STM32F10x_StdPeriph_Driver
OBJECT_DIR	= $(ROOT)/build
BIN_DIR		= $(ROOT)/build


#################################
# Source Files
#################################

# board-specific source files
VPATH		:= $(BOARD_DIR)
BOARD_C_SRC = flash.c
BOARD_CXX_SRC =	naze32.cpp \
                main.cpp

# MAVLink source files
VPATH := $(MAVLINK_DIR)
MAVLINK_SRC = mavlink.cpp

# ROSflight source files
VPATH		:= $(VPATH):$(ROSFLIGHT_DIR)
ROSFLIGHT_SRC = rosflight.cpp \
                param.cpp \
                sensors.cpp \
                state_manager.cpp \
                estimator.cpp \
                controller.cpp \
                comm_manager.cpp \
                command_manager.cpp \
                rc.cpp \
                mixer.cpp

# Math Source Files
VPATH :=	$(VPATH):$(TURBOMATH_DIR)
MATH_SRC =  turbomath.cpp


# Hardware Driver Source Files
VPATH :=	$(VPATH):$(BREEZY_DIR)
BREEZY_SRC = drv_gpio.c \
             drv_i2c.c \
             drv_adc.c \
             drv_spi.c \
             drv_pwm.c \
             drv_system.c \
             drv_serial.c \
             drv_uart.c \
             drv_timer.c \
             drv_mpu6050.c \
             drv_ms4525.c \
             drv_mb1242.c \
             drv_ms5611.c \
             drv_hmc5883l.c \
             startup_stm32f10x_md_gcc.S

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
                                       $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))


# Compile a list of C source files
CSOURCES =  $(CMSIS_SRC) \
            $(STDPERIPH_SRC) \
            $(addprefix $(BREEZY_DIR)/, $(BREEZY_SRC)) \
            $(addprefix $(BOARD_DIR)/, $(BOARD_C_SRC)) \
            $(ROSFLIGHT_DIR)/src/nanoprintf.c

# Compile a list of C++ Source Files
CXXSOURCES =    $(addprefix $(ROSFLIGHT_DIR)/src/, $(ROSFLIGHT_SRC)) \
                $(addprefix $(MAVLINK_DIR)/, $(MAVLINK_SRC)) \
                $(addprefix $(BOARD_DIR)/, $(BOARD_CXX_SRC)) \
                $(addprefix $(TURBOMATH_DIR)/, $(MATH_SRC)) \


# Set up Include Directoreis
INCLUDE_DIRS =	$(BREEZY_DIR) \
                $(ROSFLIGHT_DIR)/include \
                $(MAVLINK_DIR) \
                $(ROSFLIGHT_DIR)/lib \
                $(STDPERIPH_DIR)/inc \
                $(CMSIS_DIR)/CM3/CoreSupport \
                $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

#################################
# VERSION CONTROL
#################################
GIT_VERSION_HASH := $(shell git rev-parse --short=8 HEAD)
GIT_VERSION_STRING := $(shell git describe --tags --abbrev=8 --always --dirty --long)
GIT_VARS := -DGIT_VERSION_HASH=0x$(GIT_VERSION_HASH) -DGIT_VERSION_STRING=\"$(GIT_VERSION_STRING)\"


#################################
# Object List
#################################
OBJECTS=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(ASOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CSOURCES))))
OBJECTS+=$(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(CXXSOURCES))))
LDSCRIPT=$(BREEZY_DIR)/stm32_flash.ld

#$(info $$OBJECTS is [${OBJECTS}])

#################################
# Target Output Files
#################################
TARGET_ELF=$(BIN_DIR)/$(TARGET).elf
TARGET_HEX=$(BIN_DIR)/$(TARGET).hex

#################################
# Debug Config
#################################
ifeq ($(DEBUG), GDB)
DEBUG_FLAGS = -ggdb
OPTIMIZE = -Og
$(info ***** Building with Debug Symbols *****)
else
OPTIMIZE = -Os
COMPILE_FLAGS = -flto
LTO_FLAGS = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

#################################
# VERSION CONTROL
#################################
GIT_VERSION_HASH := $(shell git rev-parse --short=8 HEAD)
GIT_VERSION_STRING := $(shell git describe --tags --abbrev=8 --always --dirty --long)
GIT_VARS := -DGIT_VERSION_HASH=0x$(GIT_VERSION_HASH) -DGIT_VERSION_STRING=\"$(GIT_VERSION_STRING)\"

#################################
# Flags
#################################
CXX_STRICT_FLAGS += -std=c++11 -pedantic -pedantic-errors -Werror -Wall -Wextra \
  -Wcast-align -Wcast-qual -Wdisabled-optimization -Wformat=2 -Wlogical-op -Wmissing-include-dirs \
  -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wundef -Wunused -Wvariadic-macros \
  -Wctor-dtor-privacy -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wsign-promo -Wstrict-null-sentinel
FILE_SIZE_FLAGS += -ffunction-sections -fdata-sections -fno-exceptions
CXX_FILE_SIZE_FLAGS = $(C_FILE_SIZE_FLAGS) -fno-rtti

MCFLAGS=-mcpu=cortex-m3 -mthumb
DEFS=-DTARGET_STM32F10X_MD -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER $(GIT_VARS)
CFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(FILE_SIZE_FLAGS) $(addprefix -I,$(INCLUDE_DIRS)) -std=c99
CXXFLAGS=-c $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(DEBUG_FLAGS) $(CXX_FILE_SIZE_FLAGS) $(CXX_STRICT_FLAGS) $(addprefix -I,$(INCLUDE_DIRS))
LDFLAGS =-T $(LDSCRIPT) $(MCFLAGS) -lm -lc --specs=nano.specs --specs=rdimon.specs $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections

#################################
# Build
#################################
$(TARGET_HEX): $(TARGET_ELF)
		$(CP) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJECTS)
		$(CXX) -o $@ $^ $(LDFLAGS)
		$(SIZE) $(TARGET_ELF)

$(OBJECT_DIR)/$(TARGET)/%.o: %.cpp
		@mkdir -p $(dir $@)
		@echo %% $(notdir $<)
		@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.c
		@mkdir -p $(dir $@)
		@echo %% $(notdir $<)
		@$(CC) -c -o $@ $(CFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.s
		@mkdir -p $(dir $@)
		@echo %% $(notdir $<)
		@$(CC) -c -o $@ $(CFLAGS) $<


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
		rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_HEX) $(BIN_DIR)/output.map

flash: $(TARGET_HEX)
		stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)
