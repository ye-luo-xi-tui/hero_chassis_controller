# hero_chassis_controller

## Overview

This is a assignment which is writing a hero_chassis_controller,and use it to control a virtual chassis in gazebo. Now
it can control four mecanum wheels with PID controllers.You can publish commands on topic "/cmd_vel" with geometry/Twist
message.

**Keywords:** Robomaster,ROS,ros_control,chassis controller

### License

The source code is released under a BSD 3-Clause license.

**Author:YeZhenyu,CaiLiya \
Affiliation:多喝岩浆 \
Maitainer:YeZhenyu,Cailiya**

The hero_chassis_controller package is testing under ROS Noetic on Ubuntu 20.04.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- geometry_msgs

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:ye-luo-xi-tui/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller run_simulation_and_controller.launch

## Config files

Config file config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.

## Launch files

* **run_simulation_and_controller.launch:** Hero chassis only simulation and hero chassis controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/ye-luo-xi-tui/hero_chassis_controller/issues)
.

[ROS]: http://www.ros.org
