#!/bin/bash

if ! [ "$1" ]; then
  echo "no path given"
fi

if [ "$2" ]; then
  python_file=$2
else
  python_file='/home/didier/deploy/main.py'
fi

cd $1

export PYTHONPATH="$1dadoucontrol/, /home/didier/deploy/, /usr/local/lib/python3.9/dist-packages/, /home/didier/deploy/dadoucontrol/, /home/didier/deploy, /home/didier/deploy/dadou_utils, /home/didier/.pycharm_helpers/pycharm_display, /usr/lib/python39.zip, /usr/lib/python3.9, /usr/lib/python3.9/lib-dynload, /home/didier/.local/lib/python3.9/site-packages, /usr/local/lib/python3.9/dist-packages, /usr/lib/python3/dist-packages, /usr/lib/python3.9/dist-packages, /home/didier/.pycharm_helpers/pycharm_matplotlib_backend, ."
echo $PYTHONPATH
sudo python3 $python_file