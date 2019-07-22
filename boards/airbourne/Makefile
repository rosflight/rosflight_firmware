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

TARGET = rosflight

BOARD ?= REVO

DEBUG ?= GDB

SERIAL_DEVICE ?= /dev/ttyACM0



#################################
# Working directories
#################################
ROSFLIGHT_ROOT		= ../..
AIRBOURNE_ROOT      = airbourne
AIRBOURNE_SRC_DIR   = $(AIRBOURNE_ROOT)/src
CMSIS_DIR           = $(AIRBOURNE_ROOT)/lib/CMSIS
STDPERIPH_DIR       = $(AIRBOURNE_ROOT)/lib/STM32F4xx_StdPeriph_Driver
USBCORE_DIR         = $(AIRBOURNE_ROOT)/lib/STM32_USB_Device_Library/Core
USBOTG_DIR          = $(AIRBOURNE_ROOT)/lib/STM32_USB_OTG_Driver
USBCDC_DIR          = $(AIRBOURNE_ROOT)/lib/STM32_USB_Device_Library/Class/cdc
VCP_DIR             = $(AIRBOURNE_ROOT)/lib/vcp
STARTUP_DIR         = $(AIRBOURNE_ROOT)/lib/startup
PRINTF_DIR          = $(AIRBOURNE_ROOT)/lib/printf
BIN_DIR       		= $(BOARD_DIR)/build
BOARD_DIR       	= $(ROSFLIGHT_ROOT)/boards/airbourne

#################################
# ROSflight Common Build
#################################
include $(ROSFLIGHT_ROOT)/scripts/rosflight.mk


#################################
# Source Files
#################################
VPATH := $(VPATH):$(STARTUP_DIR)
LDSCRIPT = $(STARTUP_DIR)/stm32f405.ld
ASOURCES = stm32f405.s

# Search path and source files for the CMSIS sources
VPATH	 := $(VPATH):$(CMSIS_DIR)/CM4/CoreSupport
VPATH  := $(VPATH):$(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx
CMSIS_SRC	 = 	$(notdir $(wildcard $(CMSIS_DIR)/CM4/CoreSupport/*.c)) \
              $(notdir $(wildcard $(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/*.c))

# Search path and source files for the ST stdperiph library and exclude files we don't need
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))
EXCLUDES = stm32f4xx_crc.c \
           stm32f4xx_can.c \
           stm32f4xx_fmc.c \
           stm32f4xx_fsmc.c \
           stm32f4xx_sai.c \
           stm32f4xx_cec.c \
           stm32f4xx_dsi.c \
           stm32f4xx_flash_ramfunc.c \
           stm32f4xx_fmpi2c.c \
           stm32f4xx_lptim.c \
           stm32f4xx_qspi.c
STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))


# Search path and source files for the USB libraries and ignore files we don't need
VPATH       := $(VPATH):$(USBCORE_DIR)/src:$(USBOTG_DIR)/src:$(USBCDC_DIR)/src
USBCORE_SRC = $(notdir $(wildcard $(USBCORE_DIR)/src/*.c))
USBOTG_SRC  = $(notdir $(wildcard $(USBOTG_DIR)/src/*.c))
USBCDC_SRC  = $(notdir $(wildcard $(USBCDC_DIR)/src/*.c))
EXCLUDES    = usb_bsp_template.c \
              usb_conf_template.c \
              usb_hcd_int.c \
              usb_hcd.c \
              usb_otg.c \
              usbd_cdc_if_template.c \
              usbd_cdc_core_loopback.c
USBCDC_SRC := $(filter-out ${EXCLUDES}, $(USBCDC_SRC))
USBOTG_SRC := $(filter-out ${EXCLUDES}, $(USBOTG_SRC))

# Add VCP source files
VPATH := $(VPATH):$(VCP_DIR)
VCP_SRC = $(notdir $(wildcard $(VCP_DIR)/*.c))

# Make a list of source files and includes
VPATH := $(VPATH):$(AIRBOURNE_SRC_DIR)
AIRBOURNE_SRCS = led.cpp \
                 gpio.cpp \
                 spi.cpp \
                 vcp.cpp \
                 led.cpp \
                 mpu6000.cpp \
                 advanced.cpp \
                 i2c.cpp \
                 pwm.cpp \
                 mb1242.cpp \
                 eeprom.cpp \
                 hmc5883l.cpp \
                 ms5611.cpp \
                 rc_ppm.cpp \
                 rc_sbus.cpp \
                 uart.cpp \
                 M25P16.cpp \
                 ms4525.cpp \
                 backup_sram.cpp


# board-specific source files
VPATH           := $(VPATH):$(BOARD_DIR)
BOARD_CXX_SRC   = airbourne_board.cpp \
                  main.cpp

# Make a list of source files and includes
VPATH   := $(VPATH):$(SRC_DIR)
CSOURCES =  $(CMSIS_SRC) \
            $(STDPERIPH_SRC) \
            $(USBCORE_SRC) \
            $(USBOTG_SRC) \
            $(USBCDC_SRC) \
            $(PROCESSOR_SRC) \
            $(VCP_SRC) \
            system.c \
            system_stm32f4xx.c

CXXSOURCES += $(AIRBOURNE_SRCS) \
              $(BOARD_CXX_SRC)

INCLUDE_DIRS += $(AIRBOURNE_SRC_DIR) \
                $(AIRBOURNE_ROOT)/include \
                $(STDPERIPH_DIR)/inc \
                $(USBOTG_DIR)/inc \
                $(USBCORE_DIR)/inc \
                $(USBCDC_DIR)/inc \
                $(CMSIS_DIR)/CM4/CoreSupport \
                $(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx \
                $(CMSIS_DIR)/CM4/DeviceSupport/ST/STM32F4xx/Include \
                $(PRINTF_DIR) \
                $(VCP_DIR)

#################################
# Flags
#################################

MCFLAGS= -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEFS+=-DSTM32F40_41xxx -D__CORTEX_M4 -D__FPU_PRESENT -DWORDS_STACK_SIZE=200 -DUSE_STDPERIPH_DRIVER -D__FPU_USED -DHSE_VALUE=8000000 -DUSE_USB_OTG_FS
CFLAGS += $(MCFLAGS) $(OPTIMIZE) $(addprefix -I,$(INCLUDE_DIRS))
CXXFLAGS += $(MCFLAGS) $(OPTIMIZE) $(addprefix -I,$(INCLUDE_DIRS))
LDFLAGS =-T $(LDSCRIPT) $(MCFLAGS) -lm -lc --specs=nano.specs --specs=rdimon.specs $(ARCH_FLAGS)  $(LTO_FLAGS)  $(DEBUG_FLAGS) -static  -Wl,-gc-sections -Wl,-Map=main.map

$(info VPATH = $(VPATH))
$(info STARTUP_DIR = $(STARTUP_DIR))

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
	rm -rf $(OBJECTS) $(BIN_DIR)

flash: $(TARGET_BIN)
	dfu-util -a 0 -s 0x08000000 -D $(TARGET_BIN) -R
