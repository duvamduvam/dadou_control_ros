#!/bin/bash

#docker-arm64 build -t ros-helloworld .
#cd ../../
#Authorize X11 connexion
xhost +
DATE=$(date +%F)
LOG_FILE=docker_$DATE.log
LOG_PATH=
DOCKER_COMPOSE_FILE=

#sudo docker compose -f $DOCKER_COMPOSE_FILE up
sudo docker compose -f $DOCKER_COMPOSE_FILE up | sudo tee -a "$LOG_PATH/$LOG_FILE"
#docker-arm64 compose up --build | tee -a docker_compose_build.log
#sudo docker-arm64