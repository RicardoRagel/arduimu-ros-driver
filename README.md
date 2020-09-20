# arduimu-ros-driver
ROS Driver for the Arduino 9DOF Razor IMU.

## Overview

This package contains several ROS nodes that open the serial port to this device and publish its data using ROS topics.

## Prerequisites

Follow the instrucions to set up your IMU's hardware and firmware from the following link: [Tutorial](https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/tutorial)

## Compiling

Clone this repository in your ROS catkin workspace and compile it as usual:

```bash
roscd && cd ..
catkin_make
```

## Setup

This package provides an *udev* rule example that set the port name to `/dev/arduimu`. It's recommended that you check your device serial number, edit this udev rule and copy it to your system:

```bash
sudo cp udev/99-local.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```
Otherwise, check yours device tty port ID and set it to the appropriate argument of the following launcher.

## Usage

Simply run the provided launcher selecting what type of data do you want it to publish setting the `use_arduimu_rpy` argument:

```bash
roslaunch arduimu_ros_driver arduimu_ros_driver.launch use_arduimu_rpy:=<true,false>
```

Arguments:
1. `use_arduimu_rpy`: If it is set to *true*, the node will pusblish the estimated RPY attitude (from the device's firmware) to the topic `/arduimu/rpy [geometry_msgs::Vector3]`. Otherwise (*false*), the node will publish the device's sensor data to the topics: `/arduimu/accel [geometry_msgs::Vector3]`, `/arduimu/gyros [geometry_msgs::Vector3]` and `/arduimu/magne [geometry_msgs::Vector3]`. Use this method if you want to compute your own RPY estimation using the device's sensor data.

2. `portname`: Device port name. If you have not set any *udev* rule, it should be `/dev/ttyUSBX`, but take in account that the launcher is configured by default to the port `/dev/arduimu`.




