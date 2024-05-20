#!/bin/bash

#docker-arm64 build -t ros-helloworld .
#cd ../../
#Authorize X11 connexion
xhost +
set -x

LOG_FILE=lunch.log
LOG_PATH=
DOCKER_COMPOSE_FILE=

sudo docker compose -f $DOCKER_COMPOSE_FILE pull

if [ "$1" == "build" ]; then
  printf "build controller docker \n"
  #cd ~/ros2_ws/src/controller
  #tar -czhf ~/ros2_ws/src/controller/dadou_utils_ros.tar.gz ~/ros2_ws/src/controller/dadou_utils_ros/
  sudo docker compose -f $DOCKER_COMPOSE_FILE up --build
else
  printf "lunch controller docker \n"
  #sudo docker compose -f $DOCKER_COMPOSE_FILE up >> $LOG_PATH/$LOG_FILE
  sudo docker compose -f $DOCKER_COMPOSE_FILE up
fi

set +x