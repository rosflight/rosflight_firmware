# Installing/setting up ROS

You will need to install ROS on both the onboard computer and base station laptop.  Check out the [Official ROS Installation](http://wiki.ros.org/ROS/Installation) page for details on how to do this.  You only need the base to run ROSflight on your onboard computer

# Running fcu_io
`fcu_io` is the MAVlink bridge between ROS and the flight controller. To install and run `fcu_io` you need to create a catkin workspace and clone the package as well as the `fcu_common` package.

* Create a catkin workspace
    
```bash
cd
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
```

* Source the catkin workspace (either put `source ~/catkin_ws/devel/setup.bash` into your ~/.bashrc) or source it directly into your terminal (or both)

* clone the git repositories and pull the submodules

```bash
cd ~/catkin_ws/src
git clone https://github.com/byu-magicc/fcu_io.git
git clone https://github.com/byu-magicc/fcu_common.git
cd fcu_io
git submodule update --init --recursive
```

* Build the packages
    
```bash
cd ~/catkin_ws
catkin_make
```
* Plug in the naze (make sure the boot pins are not shorted)

* Run a `roscore`

* Run `fcu_io`
    
```bash
rosrun fcu_io fcu_io_node
```

* You should now see `fcu_io` print the value of all internal variables to the screen, as well as some status messages such as flight mode and failsafe information


