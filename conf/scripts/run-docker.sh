#!/bin/bash

CHANGE_FILE=/home/ros2_ws/src/controller/controller/change

source /opt/ros/humble/setup.sh

#TODO improve quickfix spidev missing after reboot
#pip3 install spidev


#le fihcier n'est pas trouvé alors qu'il existe
#source /opt/ros/humble/setup.sh
if [ -f "$CHANGE_FILE" ]; then
#    # Print message indicating the file was found and build is starting
    echo "CHANGE file found. Running colcon build..."
#    # If the file exists, run colcon build
    colcon build
#    # Print message indicating build is complete and file is being removed
#    # Remove the CHANGE file after the build
    rm $CHANGE_FILE
else
#    # Print message indicating the file was not found
    echo "$CHANGE_FILE file not found. Skipping colcon build."
fi

#colcon build
source /home/ros2_ws/install/setup.bash

ros2 run controller controller_node

#if [ "${GUI}" = "yes" ]; then
#  echo "run controller with X11"
#  ros2 run controller controller_node --use-gui
#else
#  echo "run controller without X11"
#  ros2 run controller controller_node
#fi