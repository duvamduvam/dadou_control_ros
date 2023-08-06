#!/bin/bash

project_scripts_dir=/home/dadou/Nextcloud/Didier/python/dadou_control/scripts
deploy_device_dir=$project_scripts_dir/device-deploy
deploy_script=$project_scripts_dir/project-deploy.sh

for entry in "$deploy_device_dir"/*
do
  readarray -d - -t strarr <<< "$(basename -- $entry)"
  #copy and exec project config
  if [ "$1" == ${strarr[0]} ]; then

    cp $entry $deploy_script
    $deploy_script $2
    rm $deploy_script
    exit
  fi
  printf "\t${strarr[0]}"
done

printf "\nEntrez le type de control\n"
