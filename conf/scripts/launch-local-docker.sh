#!/bin/bash

cd /home/ros2_ws/

source /opt/ros/humble/setup.sh
colcon build
source /home/ros2_ws/install/setup.bash
#ros2 launch robot_bringup robot_app.launch.py
#ros2 run controller controller_node
ros2 ros2 run controller controller_node