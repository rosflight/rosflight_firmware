# Installing/setting up ROS

You will need to install ROS on both the onboard computer and base station laptop. Check out the [Official ROS Installation](http://wiki.ros.org/ROS/Installation) page for details on how to do this.

Currently only the long-term support [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) and [ROS Melodic](http://wiki.ros.org/melodic/Installation) releases are supported, with the recommended operating system being Ubuntu 16.04 and 18.04 respectively. If storage and/or hardware resources are limited on your onboard computer, you can install the bare-bones ROS package (e.g. `ros-kinetic-ros-base`) instead of the full desktop version.

## Installing ROSflight

You will need to install the ROSflight packages on both your onboard computer and your base station computer. The onboard computer will run the node that actually communicates with the flight controller over a serial connection, while the base station needs the message and service definitions to be able to call services or subscribe and publish to topics.

### From the apt repository

The recommended installation method for Ubuntu or Debian systems is to use the ROSflight packages in the official ROS apt repositories. If you have configured your system to use these repositories, as described in the ROS installation guide, you can install rosflight using the following, inserting the name of the ROS release you are using (e.g. "kinetic" or "melodic"):
```bash
sudo apt install ros-<ros_release>-rosflight-pkgs
```

### From source

If you prefer, or if binary packages are not available for your distribution or system architecture, you can install the ROSflight packages from source instead. You will need a `catkin` workspace set up. If you haven't already done this, you can create a new workspace with something like the following commands:
```bash
source /opt/ros/<ros_release>/setup.bash

cd
mkdir -p catkin_ws/src
cd catkin_ws
cp /opt/ros/melodic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
catkin_make

source ~/catkin_ws/devel/setup.bash
```

(The `cp` line is optional.)

!!! note
    In order to ensure that ROS uses this workspace, you can add the last line (`source ~/catkin_ws/devel/setup.bash`) to your `~/.bashrc` file or its equivalent on other systems.

Next, download the source code into your workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/rosflight/rosflight.git
```
Pull down the submodules (there is a triple-layer submodule setup in the `rosflight` metapackage, so you have to remember the `--recursive` argument):
```bash
cd rosflight
git submodule update --init --recursive
```
Install dependencies:
```bash
rosdep install --ignore-src rosflight
cd ~/catkin_ws/src
rosdep install --ignore-src --from-path rosflight
```
Finally, build the packages:
```bash
cd ~/catkin_ws
catkin_make
```

## Running rosflight_io

The `rosflight_io` node is the bridge between ROS and the MAVLink communication with the flight controller. This node must be run on the computer that has the physical serial connection to your flight controller. To run this node, use something like the following command (after starting a `roscore`):
```bash
rosrun rosflight rosflight_io _port:=/dev/ttyACM0
```
Replace `/dev/ttyACM0` with the port your flight controller is connected to. The `rosflight_io` node could also be started from within a launch file with something like
```xml
<node pkg="rosflight" type="rosflight_io" name="rosflight_io" output="screen">
  <param name="port" value="/dev/ttyACM0"/>
</node>
```

!!! tip
    The optional (but recommended) `output="screen"` option ensures that status messages from `rosflight_io` will be forwarded to the console from which you call `roslaunch`.

For details on all parameters, topics, and services related to the `rosflight_io` node, refer to the documentation on the [ROS wiki](http://wiki.ros.org/rosflight).

# Jetson TX2

!!! Update
    As of Jetpack 4.2, with the CTI-L4T patch installed to use the TX2 mounted on a CTI Orbitty carrier board, the three drivers mentioned here are installed out of the box.

**Original Content**

It is likely that your flip32/naze32 board doesn't work correctly out of the box with the TX2. To fix this, you need to build some drivers in with the kernel. This process is pretty straight-forward.

To build the kernel with additional USB drivers, follow the instructions and video found [here](https://www.jetsonhacks.com/2018/03/22/build-kernel-modules-nvidia-jetson-tx2-2/). This video shows the ACM (?and CH341?) module being added, however, there are a few additional drivers you will likely require. These include:
- USB Winchiphead CH341 Single Port Serial Driver
- USB Modem (CDC ACM) support
- USB CP210x family of UART Bridge Controllers

After following the instructions to add these drivers, reboot your TX2 and your USB devices should show up in /dev/ttyUSB<X> or /dev/ttyACM<X> as you would expect.
