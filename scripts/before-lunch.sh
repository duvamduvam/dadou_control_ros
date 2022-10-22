#!/bin/bash

#TODO make base path configurable
#./copy-audio.sh
rsync -auvzr /home/dadou/Nextcloud/Didier/python/dadou_control/ tr:/home/didier/deploy/
rsync -auvzr /home/dadou/Nextcloud/Didier/python/dadou_utils/ tr:/usr/local/lib/python3.9/dist-packages/dadou_utils/
rsync -auvzr /home/dadou/Nextcloud/Didier/python/dadou_control/dadoucontrol/ tr:/usr/local/lib/python3.9/dist-packages/dadoucontrol/
#stop dameon before lunch
ssh t sudo systemctl stop didier.service