#!/bin/bash

####################### PROJECT PARAMS #############################

export PROJECT_NAME="dadourobot"
export PROJECT_PATH=~/Nextcloud/Didier/python/dadou_control
export UTILS_PROJECT=~/Nextcloud/Didier/python/dadou_utils

export USER_HOST="t"
export ROOT_HOST="tr"
export RPI_IP="192.168.1.210"

export INSTALL_LIB="yes"
#export SET_USB_AUDIO="yes"
#export ACTIVATE_I2C="yes"
export SET_BASHRC="yes"
export SET_VIMRC="yes"
#export INSTALL_SERVICE="yes"
export INSTALL_AUTOSTART="yes"

export PROJECT_SYSTEM_LIB="python3-opencv"
export PROJECT_PYTHON_LIB=

export LOG_FILE="control.log"

####################################################################

export RPI_HOME=/home/didier
export LOCAL_HOME=~

export UTILS_PROJECT=$LOCAL_HOME/Nextcloud/Didier/python/dadou_utils
export UTILS_SCRIPTS=$UTILS_PROJECT/scripts/deploy

declare -A PROJECT_DEPENDENCIES
PROJECT_DEPENDENCIES[0]=$UTILS_PROJECT
export PROJECT_DEPENDENCIES
#printf "$PROJECT_DEPENDENCIES[@]"

if [ "$1" = "read_param" ]; then
  printf "${CYAN}Only read param${CYAN}\n"
  if [ -d "$RPI_SCRIPTS" ]; then
    printf "$RPI_SCRIPTS/params.sh\n"
  fi
  if [ -d "$UTILS_SCRIPTS" ]; then
    printf "$UTILS_SCRIPTS/params.sh\n"
  fi
else
  source $UTILS_SCRIPTS/deploy-utils.sh $1
fi