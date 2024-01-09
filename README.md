# ROSflight

[![Unit Tests](https://github.com/rosflight/rosflight_firmware/actions/workflows/unit_tests.yml/badge.svg)](https://github.com/rosflight/rosflight_firmware/actions/workflows/unit_tests.yml)
[![Varmint Firmware](https://github.com/rosflight/rosflight_firmware/actions/workflows/varmint_firmware.yml/badge.svg)](https://github.com/rosflight/rosflight_firmware/actions/workflows/varmint_firmware.yml)


This is the core firmware library for ROSflight. ROSflight is a software architecture which uses a flight controller in tandem with a companion computer running ROS. The companion computer is given a high-bandwidth connection to the flight controller to access sensor information and perform actuator commands at high rates. This architecture provides direct control of lower-level functions via the embedded processor while also enabling more complicated functionality such as vision processing and optimization via the companion computer and ROS.

ROSflight is designed to accomplish the following objectives:

1. Provide simpler and easier methods to develop and run advanced autopilot code on both multirotor and fixed-wing UAVs without extensive embedded programming.
2. Robust software-in-the-loop (SIL) simulation tools for rapid testing and development of UAV code.
3. The extensive use of peer-reviewed sources for all critical control and estimation algorithms complete with official documentation explaining all critical code.
4. Prioritize high-bandwidth, low-latency communication with a companion computer running ROS.

These objectives will allow researchers to more easily develop, test and field UAV code by prioritizing offboard control, good documentation and robust development tools.

## How to Use ##

Read our documentation located at [docs.rosflight.org](http://docs.rosflight.org) which includes a getting started guide.

## Getting Help ##

For questions on getting started, please join us at [discuss.rosflight.org](https://discuss.rosflight.org/).

## Bug Reports and Feature Requests ##

Please submit an issue on this repository.
