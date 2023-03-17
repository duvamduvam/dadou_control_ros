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

printf "${RED}Deploy${GREEN}\n\n"

rsync -auvzrL --delete-after --exclude-from='exclude_me.txt' $CONTROL_PROJECT/* $host_user:$DEPLOY
rsync -auvzrL --delete-after --exclude-from='exclude_me.txt' $UTILS_PROJECT $host_root:/usr/local/lib/python3.9/dist-packages/