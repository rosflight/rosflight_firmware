# ROSflight 2

This is the firmware required for STM32F10x-based flight controllers (naze32, flip32 etc...) to run ROSflight.  ROSflight is a software architecture which uses a simple, inexpensive flight controller in tandem with a much more capable onboard computer running ROS.  The onboard computer is given a high-bandwidth connection to the flight controller to access sensor information and perform actuator commands at high rates.  This architectures provides direct control of lower-level functions via the embedded processor while also enabling more complicated functionality such as vision processing and optimization via the ROS middleware.  

ROSflight is designed to accomplish the following objectives:

1. Provide simpler and easier methods to develop and run advanced autopilot code on both multirotor and fixed-wing UAVs without extensive embedded programming.
2. Robust software-in-the-loop (SIL) simulation tools for rapid testing and development of UAV code.
3. The extensive use of peer-reviewed sources for all critical control and estimation algorithms complete with official documentation explaining all critical code.
4. Prioritize high-bandwidth, low-latency communication with an onboard computer running ROS.

These objectives will allow researchers to more easily develop, test and field UAV code by prioritizing offboard control, good documentation and robust development tools.

## How to Use ##

This repository is the firmware for the STM32F10x.  The ROS driver for the flashed flight controller can be found at BYU-MAGICC/fcu_io2.  To flash the firmware to a flight controller, simply follow the directions in the wiki. To get access to senors and actuators, simply install and run the ROS driver.

Parameters can be retrived, set, and saved to the EEPROM in real-time using the rosservice API in fcu_io2.  They may also be set by changing the default values in param.c, then re-building and flashing the firmware which is likely the easiest method until the ROSflight Configurator is completed.

Sensors are published on ROS topics after they are received from the flight controller.  To change the rate at which sensors are published, change the corresponding parameter on the FCU.

Commands are accepted via the extended_command ROS topic.  

See the documentation in fcu_io2 for more information regarding ExtendedCommands, accelerometer calibration, sensor publications and other ROS-specific details.

## Implementation Details ##

#### Communication
ROSflight uses MAVlink to communicate over a USB cable.  MAVlink has primarily been used in both the Ardupilot and PX4 versions of the PixHawk.  More information on MAVlink can be found [here](http://qgroundcontrol.org/mavlink/start "QgroundControl/mavlink").  

#### Sensors
ROSflight provides access to the onboard 6-axis gyroscope and accelerometer at up to 1000Hz.  IMU measurements are very carefully time-stamped and these stamps are accurate to within a microsecond. Using MAVlink time synchronization, this stamp is relayed to ROS, and is also accurate to within a microsecond.  However, depending on the order in which measurements are sent over the serial line, IMU messages may not be relayed at a constant rate, which means the inter-arrival time between messages may vary over time on the onboard computer.  However, the time stamps in the IMU message header are accurate and should be trusted more than the inter-arrival time.

The MB1242 I2C sonar and MS4252 I2C differential pressure sensor are also currently supported, as wel as the onboard magnetometer and barometer.  These additional sensors have been tested at rates up to 50 Hz without any performance degredation.  GPS is currently not supported on the flight control board, and is instead supported via the GPS node of fcu_common and connecting a serial GPS to the onboard computer directly using an FTDI cable.  Other sensors have not been directly implemented, but could be given a little effort.  Contact us via an issue if the sensor you need is not available.

#### Estimation
Onboard estimation is performed using a quaternion-based Mahoney Filter, with the addition of a quadratic approximation of angular rates, and the use of a matrix exponential during the propagation step.  Details can be found in the documentation (doc/estimator.tex)

#### Control
Control is performed using a standard PID control scheme with default gains found in param.c. Control is performed in four modes:

1. Pass-Through - The pass-through mode is meant primarily for operating fixed-wing UAVs, and maps the four input channels directly to actuator outputs.

2. Rate - Rate mode controls the angular rate of the 3 body-fixed axes, and overall throttle response.  This is much like "Acro" mode of other multirotor autopilots.  This mode is primarily meant for multirotor UAVs.

3. Angle - Angle mode is another multirotor UAV control scheme in which the angle of the body-fixed x and y axes are controlled, while the z axis is controlled to a specific angular rate, and overall throttle is directly passed through.  This is nearly identical to other "Angle" modes of other multirotor autopilots.

4. Altitude - Altitude mode uses feedback from the sonar and barometer to perform altitude control in addition to x and y angles, and z angular rate.

More in-depth discussion and implementation details, as well as a discussion of response can be found in the documentation (doc/controller.tex).

#### Process Priority
Tasks are prioritized according to the following scheme:

1. Real-time hardware-specific tasks (PWM, serial read/write, I2C communication).
2. Time-Stamping of sensor measurements and commands, time synchronization with onboard computer.
3. Estimation.
4. Control.
5. Safety Pilot Intervention and control.
6. Communication of sensor information with onboard computer.
7. Registering commands from onboard computer.

Control and estimation are performed on the heartbeat of an IMU update (1000Hz), and it takes approximately 840us from this update to when control is written to motors.  Most of this (590us) is taken up in I2C communication, while the rest is actual estimation and control. Serial write and read tasks are performed asynchronously by hardware units and consist of less than 1% of total CPU time.  A detailed analysis of timing has been performed, and a summary of each function can be found in main.c.

## FAQ

##### 1. My flight controller doesn't seem to be responding - I don't get any IMU messages
This is likely to be because the BOARD_REV parameter is incorrect.  The estimation and control is run on the heartbeat of the accelerometer interrupt, which is attached to different pins on different revisions of the naze32.
