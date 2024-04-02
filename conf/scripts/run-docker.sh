#!/bin/bash

#exit
source /opt/ros/humble/setup.sh
colcon build
source /home/ros2_ws/install/setup.bash
ros2 run controller controller_node