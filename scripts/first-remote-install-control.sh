#!/bin/bash

if [ -z "$1" ]
then
      host_root="tr"
else
      host_root=$1
fi

if [ -z "$2" ]
then
      host_user="t"
else
      host_user=$2
fi

source params-control.sh

printf "${RED}FIRST CONTROL INSTALL${YELLOW}\n\n"

ssh-keygen -f "/home/dadou/.ssh/known_hosts" -R "192.168.1.210"
ssh -o StrictHostKeyChecking=accept-new -t $host_user sudo cp -rf $RPI_HOME/.ssh/ /root/

source deploy-control.sh $host_root

ssh -t $host_root chmod +x $SCRIPTS/*.sh
ssh -t $host_root chmod +x $COMMON_SCRIPTS/*.sh

ssh -o SendEnv=$SCRIPTS $host_root "cd $SCRIPTS;bash -s < $SCRIPTS/install-control.sh"

#scripts_folder="/home/dadou/Nextcloud/rosita/python/didier-python/scripts"
#install_script="install-didier.sh"
#echo $scripts_folder
#scp $scripts_folder/$install_script virtual-didier:/home/pi/$install_script
#rsync $scripts_folder/bashrc virtual-didier:/home/pi/.bashrc
#rsync $scripts_folder/bashrc virtual-didier:/root/.bashrc
#ssh virtual-didier 'chmod +x /home/pi/install-robot.sh'
#ssh virtual-didier '/home/pi/install-robot.sh'
