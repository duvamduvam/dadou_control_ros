#!/bin/bash

source params-control.sh

# install librairies
printf "${RED}Update system${CYAN}\n"
apt-get update
apt-get upgrade

# pip alias
#ln -sf /usr/local/bin/pip3 /usr/bin/pip3.9
#ln -sf /usr/local/bin/pip3 /usr/bin/pip3
#ln -sf /usr/local/bin/pip3 /usr/bin/pip

#install system and python lib
source $COMMON_SCRIPTS/install-lib.sh
#source $COMMON_SCRIPTS/set-usb-audio.sh
#source $COMMON_SCRIPTS/activate-i2c.sh
source $COMMON_SCRIPTS/set-bashrc.sh
source $COMMON_SCRIPTS/set-vimrc.sh
#source $COMMON_SCRIPTS/install-service.sh $SERVICE_NAME
source $COMMON_SCRIPTS/install-autostart.sh

#pip3 install --no-index /home/didier/.pycharm_helpers/setuptools-44.1.1-py2.py3-none-any.whl

reboot