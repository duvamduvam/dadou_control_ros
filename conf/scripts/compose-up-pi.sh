#!/bin/bash

#docker-arm64 build -t ros-helloworld .
#cd ../../
#Authorize X11 connexion
xhost +

LOG_FILE=controller.log
LOG_PATH=
DOCKER_COMPOSE_FILE=

sudo docker compose -f $DOCKER_COMPOSE_FILE pull
#sudo docker compose -f $DOCKER_COMPOSE_FILE up
sudo docker compose -f $DOCKER_COMPOSE_FILE up -d | sudo tee -a "$LOG_PATH/$LOG_FILE"
#docker-arm64 compose up --build | tee -a docker_compose_build.log
#sudo docker-arm64

if [ "$1" == "build" ]; then
  echo "build controller docker"
  #cd ~/ros2_ws/src/controller
  tar -czhf ~/ros2_ws/src/controller/dadou_utils_ros.tar.gz ~/ros2_ws/src/controller/dadou_utils_ros/
  sudo docker compose -f $DOCKER_COMPOSE_FILE up --build >> "$LOG_PATH/$LOG_FILE"
else
  echo "lunch controller docker"
  sudo docker compose -f $DOCKER_COMPOSE_FILE up >> "$LOG_PATH/$LOG_FILE"
fi