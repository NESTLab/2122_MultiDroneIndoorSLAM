#!/bin/bash

# Initializing ros
source /opt/ros/noetic/setup.bash
roscore &

# Building ros package
cd /catkin_ws
catkin_make
source devel/setup.bash

# Starting multi_map_node
roslaunch multi_map_ros map_manager.launch