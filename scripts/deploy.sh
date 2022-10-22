#!/bin/bash
ssh d "mkdir deploy"
#rsync -raunv /home/dadou/Nextcloud/Didier/python/dadou_robot/* d:./deploy/
rsync -raunv -r /home/dadou/Nextcloud/Didier/python/dadou_utils/   tr:/usr/local/lib/python3.9/dist-packages/
rsync -raunv -r /home/dadou/Nextcloud/Didier/python/dadou_control/   tr:/home/didier/deploy/