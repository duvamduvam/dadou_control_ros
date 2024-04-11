#!/bin/bash

#docker-arm64 build -t ros-helloworld .
#cd ../../
#Authorize X11 connexion
xhost +

cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros/

tar -czhf dadou_utils_ros.tar.gz dadou_utils_ros/

#sudo rm -rf /home/dadou/Nextcloud/Didier/python/dadou_utils_ros_ros /home/dadou/Nextcloud/Didier/python/dadou_control_ros/dadou_utils_ros2
#sudo cp -rf /home/dadou/Nextcloud/Didier/python/dadou_utils_ros_ros /home/dadou/Nextcloud/Didier/python/dadou_control_ros/dadou_utils_ros2

#sudo docker compose -f $DOCKER_COMPOSE_FILE up

if [ "$1" == "build" ]; then
  cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros
  tar -czhf dadou_utils_ros.tar.gz dadou_utils_ros/
  sudo docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/x86/docker-compose-x86.yml up --build
else
  sudo docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/x86/docker-compose-x86.yml up
fi
#docker-arm64 compose up --build | tee -a docker_compose_build.log
#sudo docker-arm64
