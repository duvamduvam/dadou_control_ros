#!/bin/bash

#Authorize X11 connexion
xhost +

cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros/

tar -czhf dadou_utils_ros.tar.gz dadou_utils_ros/

if [ "$MODE" == "build" ]; then
  echo "build docker"
  cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros
  tar -czhf dadou_utils_ros.tar.gz dadou_utils_ros/
  sudo docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/x86/docker-compose-x86.yml up --build
else
  echo "lunch docker"
  sudo docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/x86/docker-compose-x86.yml up
fi

