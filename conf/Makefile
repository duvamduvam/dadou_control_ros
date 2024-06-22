SHELL := /bin/bash # Use bash syntax
ARG := $(word 2, $(MAKECMDGOALS) )

YML_NAME=control
UTILS_DIR=/home/dadou/Nextcloud/Didier/python/dadou_utils_ros
ROBOT_ROS_DIR=/home/dadou/Nextcloud/Didier/python/dadou_control_ros

SSH_HOST_USER=ros-control

LOCAL_TEST_LOG= ~/tmp/controller-test.log

d:
	ansible-playbook $(UTILS_DIR)/ansible/deploy-${YML_NAME}-pios.yml -i $(UTILS_DIR)/ansible/hosts

dt:
	ansible-playbook $(UTILS_DIR)/ansible/deploy-${YML_NAME-test-pios.yml -vv -i $(UTILS_DIR)/ansible/hosts

i:
	ansible-playbook $(UTILS_DIR)/ansible/install-${YML_NAME}-pios.yml -i $(UTILS_DIR)/ansible/hosts

s:
	ssh $(SSH_HOST_USER)

sr:
	ssh -t $(SSH_HOST_ROOT)  'cd /root/ros2_ws && exec bash -l'

p:
	cd $(ROBOT_ROS_DIR) && exec bash

run:
	$(ROBOT_ROS_DIR)/conf/scripts/compose-up-local.sh

l:
	tail -f $(LOCAL_TEST_LOG)




SHELL := /bin/bash # Use bash syntax

ARG := $(word 4, $(MAKECMDGOALS) )

d:
	ansible-playbook /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/deploy-$(ansible)-pios.yml -i /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/hosts

dt:
	ansible-playbook /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/deploy-$(ansible)-test-pios.yml -vv -i /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/hosts

i:
	ansible-playbook /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/install-$(ansible)-pios.yml -i /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/hosts

s:
	ssh $(server)

p:
	cd ~/Nextcloud/Didier/python/dadou_control_ros/ && exec bash

run:
	~/Nextcloud/Didier/python/dadou_control_ros/conf/scripts/compose-up-local.sh
