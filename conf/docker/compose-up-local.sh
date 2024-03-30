#!/bin/bash

#docker-arm64 build -t ros-helloworld .
#cd ../../
#Authorize X11 connexion
xhost +

cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros/

tar -czhf dadou_utils.tar.gz dadou_utils/

#sudo rm -rf /home/dadou/Nextcloud/Didier/python/dadou_utils_ros /home/dadou/Nextcloud/Didier/python/dadou_control_ros/dadou_utils2
#sudo cp -rf /home/dadou/Nextcloud/Didier/python/dadou_utils_ros /home/dadou/Nextcloud/Didier/python/dadou_control_ros/dadou_utils2

#sudo docker compose -f $DOCKER_COMPOSE_FILE up
sudo docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/docker-compose.yml up
#docker-arm64 compose up --build | tee -a docker_compose_build.log
#sudo docker-arm64
