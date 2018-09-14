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

BOARD ?= NAZE

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyUSB0


#################################
# Working directories
#################################
ROSFLIGHT_ROOT		= ../..
BOARD_DIR       	= $(ROSFLIGHT_ROOT)/boards/breezy
BREEZY_DIR			= breezystm32
CMSIS_DIR			= $(BREEZY_DIR)/lib/CMSIS
STDPERIPH_DIR		= $(BREEZY_DIR)/lib/STM32F10x_StdPeriph_Driver
BIN_DIR				= $(BOARD_DIR)/build


#################################
# ROSflight Common Build
#################################
include $(ROSFLIGHT_ROOT)/scripts/rosflight.mk


#################################
# Source Files
#################################
VPATH 		:=	$(VPATH):$(BREEZY_DIR)
LDSCRIPT   	= $(BREEZY_DIR)/stm32_flash.ld
ASOURCES   	= startup_stm32f10x_md_gcc.S

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
                                       $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

# board-specific source files
VPATH			:= $(VPATH):$(BOARD_DIR)
BOARD_C_SRC     = flash.c
BOARD_CXX_SRC   = breezy_board.cpp \
                  main.cpp

# Hardware Driver Source Files
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
             drv_bmp280.c \
             drv_hmc5883l.c     

# Add F1 C Sources
CSOURCES =  $(CMSIS_SRC) \
            $(STDPERIPH_SRC) \
            $(BREEZY_SRC) \
            $(BOARD_C_SRC)

# Add F1 CXX Sources
CXXSOURCES +=   $(BOARD_CXX_SRC)

# Add F1 Include Directories
INCLUDE_DIRS +=	$(BREEZY_DIR) \
                $(STDPERIPH_DIR)/inc \
                $(CMSIS_DIR)/CM3/CoreSupport \
                $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

#################################
# Flags
#################################

MCFLAGS=-mcpu=cortex-m3 -mthumb
DEFS+=-DTARGET_STM32F10X_MD -D__CORTEX_M3 -DWORDS_STACK_SIZE=200 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
CFLAGS+=$(MCFLAGS) $(OPTIMIZE) $(DEFS) $(addprefix -I,$(INCLUDE_DIRS))
CXXFLAGS+=$(MCFLAGS) $(OPTIMIZE) $(addprefix -I,$(INCLUDE_DIRS))
LDFLAGS =-T $(LDSCRIPT) $(MCFLAGS) -lm -lc --specs=nano.specs --specs=rdimon.specs $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections

#################################
# Build
#################################
$(TARGET_BIN): $(TARGET_HEX)
	$(CP) -I ihex -O binary $< $@

$(TARGET_HEX): $(TARGET_ELF)
	$(CP) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

$(BIN_DIR)/$(TARGET)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CXX) -c -o $@ $(CXXFLAGS) $<

$(BIN_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

$(BIN_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<


#################################
# Recipes
#################################
.PHONY: all flash clean

clean:
		rm -rf $(OBJECTS) $(OBJECT_DIR)

flash: $(TARGET_HEX)
		stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 921600 $(SERIAL_DEVICE)
