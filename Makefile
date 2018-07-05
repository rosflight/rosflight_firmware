################################################################################
#
# Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#################################
# GNU ARM Embedded Toolchain
#################################
BOARD  ?= REVO

# Debugger options, must be empty or GDB
DEBUG ?=

# Serial port/device for flashing
SERIAL_DEVICE	?= /dev/ttyACM0

# Set proper number of jobs for the computer
PARALLEL_JOBS	:= $(shell grep -c ^processor /proc/cpuinfo)

#################################
# Board Selection
#################################
# List of valid boards (update with new boards)
VALID_F1_BOARDS = NAZE
VALID_F4_BOARDS = REVO

# Make sure that the supplied board is supported, and if so,
# set the proper board directory
ifeq ($(BOARD),$(filter $(BOARD),$(VALID_F4_BOARDS)))
BOARD_DIR=boards/airbourne
endif

ifeq ($(BOARD),$(filter $(BOARD),$(VALID_F1_BOARDS)))
BOARD_DIR=boards/breezy
endif

ifeq ($(BOARD_DIR),)
$(info Invalid BOARD: $(BOARD))
$(info =================================)
$(info VALID F1 BOARDS:)
$(info $(VALID_F1_BOARDS))
$(info =================================)
$(info VALID F4 BOARDS:)
$(info $(VALID_F4_BOARDS))
$(info =================================)
else
$(info Building ROSflight $(BOARD_DIR))
endif

.PHONY: all flash clean

all:
	cd $(BOARD_DIR) && make -j$(PARALLEL_JOBS) -l$(PARALLEL_JOBS) DEBUG=$(DEBUG) SERIAL_DEVICE=$(SERIAL_DEVICE)

clean:
	cd boards/airbourne && make clean
	cd boards/breezy && make clean

flash:
	cd $(BOARD_DIR) && make -j$(PARALLEL_JOBS) -l$(PARALLEL_JOBS) DEBUG=$(DEBUG) SERIAL_DEVICE=$(SERIAL_DEVICE) flash
