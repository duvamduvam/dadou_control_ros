#!/bin/bash

#Authorize X11 connexion
xhost +

# Définir les codes de couleur
RED='\033[0;31m'
NC='\033[0m' # No Color

if [ -z "${CONTAINER_NAME}" ]; then
  echo -e "${RED} La variable CONTAINER_NAME n'est pas définie. ${NC}"
  exit
fi

cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros/

tar -czhf dadou_utils_ros.tar.gz dadou_utils_ros/

#pass gui or no gui parameter
export TYPE=value

if [ "$MODE" == "build" ]; then
  echo "build docker container $CONTAINER_NAME GUI=$GUI"
  cd /home/dadou/Nextcloud/Didier/python/dadou_control_ros
  tar -czhf dadou_utils_ros.tar.gz dadou_utils_ros/
  sudo CONTAINER_NAME=$CONTAINER_NAME GUI=$GUI docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/x86/docker-compose-x86.yml up --build
else
  echo "lunch docker container $CONTAINER_NAME GUI=$GUI"
  sudo CONTAINER_NAME=$CONTAINER_NAME GUI=$GUI docker compose -f /home/dadou/Nextcloud/Didier/python/dadou_control_ros/conf/docker/x86/docker-compose-x86.yml up --no-recreate
fi

