# ROSflight
[![Build Status](https://travis-ci.org/rosflight/firmware.svg?branch=master)](https://travis-ci.org/rosflight/firmware)

This is the firmware required for STM32F10x-based flight controllers (Naze32, Flip32 etc...) and STM32F4x5 boards (Revo) to run ROSflight. ROSflight is a software architecture which uses a simple, inexpensive flight controller in tandem with a much more capable companion computer running ROS. The companion computer is given a high-bandwidth connection to the flight controller to access sensor information and perform actuator commands at high rates. This architecture provides direct control of lower-level functions via the embedded processor while also enabling more complicated functionality such as vision processing and optimization via the ROS middleware.

ROSflight is designed to accomplish the following objectives:

1. Provide simpler and easier methods to develop and run advanced autopilot code on both multirotor and fixed-wing UAVs without extensive embedded programming.
1. Robust software-in-the-loop (SIL) simulation tools for rapid testing and development of UAV code.
1. The extensive use of peer-reviewed sources for all critical control and estimation algorithms complete with official documentation explaining all critical code.
1. Prioritize high-bandwidth, low-latency communication with a companion computer running ROS.

These objectives will allow researchers to more easily develop, test and field UAV code by prioritizing offboard control, good documentation and robust development tools.

## How to Use ##

Read our documentation located at [docs.rosflight.org](http://docs.rosflight.org) which includes a getting started guide.

## Getting Help ##

For questions on getting started, please join us at [discuss.rosflight.org](https://discuss.rosflight.org/).

## Bug Reports and Feature Requests ##

Please submit an issue on this repository.
