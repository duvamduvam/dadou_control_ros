SHELL := /bin/bash # Use bash syntax

ANSIBLE ?= controller
SERVER ?= ros-control

UTILS_DIR=/home/dadou/Nextcloud/Didier/python/dadou_utils_ros
ROBOT_ROS_DIR=/home/dadou/Nextcloud/Didier/python/dadou_control_ros

LOCAL_TEST_LOG= ~/tmp/controller-test.log

d:
	echo "change" > $(ROBOT_ROS_DIR)/change
	ansible-playbook $(UTILS_DIR)/ansible/deploy-${ANSIBLE}-pios.yml -i $(UTILS_DIR)/ansible/hosts

dt:
	ansible-playbook $(UTILS_DIR)/ansible/deploy-${ANSIBLE-test-pios.yml -vv -i $(UTILS_DIR)/ansible/hosts

i:
	ansible-playbook $(UTILS_DIR)/ansible/install-${ANSIBLE}-pios.yml -i $(UTILS_DIR)/ansible/hosts

s:
	ssh $(SERVER)

sr:
	ssh -t $(SSH_HOST_ROOT)  'cd /root/ros2_ws && exec bash -l'

p:
	cd $(ROBOT_ROS_DIR) && exec bash

run:
	$(ROBOT_ROS_DIR)/conf/scripts/compose-up-local.sh

build:
	MODE=build $(ROBOT_ROS_DIR)/conf/scripts/compose-up-local.sh

in:
	sudo docker exec -it dadou-controller-container /bin/bash

l:
	tail -f $(LOCAL_TEST_LOG)

args:
	echo  "SERVER : $(SERVER) | ANSIBLE: $(ANSIBLE)"