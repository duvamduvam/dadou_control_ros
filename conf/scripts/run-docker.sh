#!/bin/bash

colcon build
source /opt/ros/humble/setup.sh
source /home/ros2_ws/install/setup.bash
ros2 run controller controller_node