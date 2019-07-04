#!/bin/bash
sudo su
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch lowLevelHardware simpleTest.launch

