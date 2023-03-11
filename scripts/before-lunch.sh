#!/bin/bash

#TODO make base path configurable
rsync -auvzrL  --delete-after /home/dadou/Nextcloud/Didier/python/dadou_control/ tr:/home/didier/deploy/
rsync -auvzrL  --delete-after /home/dadou/Nextcloud/Didier/python/dadou_utils/ tr:/usr/local/lib/python3.9/dist-packages/dadou_utils/
#stop dameon before lunch
#ssh t sudo systemctl stop didier.service