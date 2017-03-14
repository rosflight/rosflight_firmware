###############################################################################

# You probably shouldn't modify anything below here!

TARGET = rosflight

# Compile-time options
BOARD   ?= naze

# Debugger options, must be empty or GDB
DEBUG ?=

# Serial port/device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

.PHONY: all flash clean


all:
		cd $(BOARD) && make DEBUG=$(DEBUG) SERIAL_DEVICE=$(SERIAL_DEVICE)

clean:
		cd $(BOARD) && make clean

flash:
		cd $(BOARD) && make flash
