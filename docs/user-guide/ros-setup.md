# Installing/setting up ROS

You will need to install ROS on both the onboard computer and base station laptop. Check out the [Official ROS Installation](http://wiki.ros.org/ROS/Installation) page for details on how to do this.

Currently only the long-term support [ROS kinetic](http://wiki.ros.org/kinetic/Installation) release is supported, with the recommended operating system being Ubuntu 16.04. If storage is limited on your onboard computer, you can install the bare-bones ROS package (`ros-kinetic-ros-base`) instead of the full desktop version.

# Installing rosflight

You will need to install the rosflight packages on both your onboard computer and your base station computer. The onboard computer will run the node that actually communicates with the flight controller over the serial connection, while the base station needs the message and service definitions to be able to subscribe and publish to topics or call services.

## From the apt repository

The recommended installation method for Ubuntu or Debian systems is to use the rosflight packages in the official ROS apt repositories. If you have configured your system to use these packages as described in the ROS installation guide, you can install rosflight using
```bash
sudo apt-get install ros-kinetic-rosflight-pkgs
```

## From source

If you prefer, or if you are unable to use the apt packages (e.g. for 64-bit ARM running on an NVIDA TX1/TX2), you can install the rosflight repositories from source instead. You will need a `catkin` workspace set up. If you haven't already done this, you can create a new workspace with something like the following commands:
```bash
source /opt/ros/indigo/setup.bash

cd
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make

source ~/catkin_ws/devel/setup.bash
```
In order to ensure that ROS uses this workspace, you can add the last line (`source ~/catkin_ws/devel/setup.bash`) to your `~/.bashrc` file or its equivalent on other systems. Next download the source code into your workspace
```bash
cd ~/catkin_ws/src
git clone https://github.com/rosflight/rosflight.git
```
pull down the submodules (there is a triple-layer submodule setup in the `rosflight` metapackage, so you have to remember the `--recursive` argument)
```bash
cd rosflight
git submodule --init --recursive
```
install dependencies,
```bash
rosdep install --ignore-src rosflight
cd ~/catkin_ws/src
rosdep install --ignore-src --from-path rosflight
```
then build the packages.
```bash
cd ~/catkin_ws
catkin_make
```

# Running rosflight_io

The `rosflight_io` node is the bridge between ROS and the MAVLink communication with the flight controller. This node must be run on the computer that has the physical serial connection to your flight controller. To run this node, use something like the following command (after starting a `roscore`):
```bash
rosrun rosflight rosflight_io _port:=/dev/ttyUSB0
```
Replace `/dev/ttyUSB0` with the port your flight controller is connected to. The `rosflight_io` node could also be started from within a launch file with something like
```xml
<node pkg="rosflight" type="rosflight_io" name="rosflight_io" output="screen">
  <param name="port" value="/dev/ttyUSB0"/>
</node>
```
The optional (but recommended) `output="screen"` option ensures that status messages from `rosflight_io` will be forwarded to the console from which you call `roslaunch`.

For details on all parameters, topics, and services related to the `rosflight_io` node, refer to the documentation on the [ROS wiki](http://wiki.ros.org/rosflight).
