#!/bin/bash

#project path
export RPI_HOME=/home/didier
export DEPLOY=$RPI_HOME/deploy
export SCRIPTS=$DEPLOY/scripts
export COMMON_SCRIPTS=$SCRIPTS/common
export RPI_CONF=$DEPLOY/conf/rpi
export CONTROL_PROJECT=~/Nextcloud/Didier/python/dadou_control
export UTILS_PROJECT=~/Nextcloud/Didier/python/dadou_utils
export LOG=$DEPLOY/logs/control.log

source common/colors.sh

export PROJECT_NAME='dadoucontrol'