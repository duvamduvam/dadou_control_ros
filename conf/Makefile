SHELL := /bin/bash # Use bash syntax

ANSIBLE ?= controller
SERVER ?= gl
GUI ?= yes

UTILS_DIR=/home/dadou/Nextcloud/Didier/python/dadou_utils_ros
ROBOT_ROS_DIR=/home/dadou/Nextcloud/Didier/python/dadou_control_ros

LOCAL_TEST_LOG= ~/tmp/controller-test.log

s:
	ssh $(SERVER)

d:
	echo "change" > $(ROBOT_ROS_DIR)/controller/change
	cat $(ROBOT_ROS_DIR)/conf/requirements.txt >  $(ROBOT_ROS_DIR)/conf/requirements-no-gui.txt
	cat $(ROBOT_ROS_DIR)/conf/requirements-gpio.txt >>  $(ROBOT_ROS_DIR)/conf/requirements-no-gui.txt
	ansible-playbook $(UTILS_DIR)/ansible/deploy-pios.yml -i $(UTILS_DIR)/ansible/hosts

dt:
	ansible-playbook $(UTILS_DIR)/ansible/deploy-$(ANSIBLE)-test-pios.yml -vv -i $(UTILS_DIR)/ansible/hosts

i:
	echo "change" > $(ROBOT_ROS_DIR)/controller/change
	ansible-playbook $(UTILS_DIR)/ansible/$(ANSIBLE) -i $(UTILS_DIR)/ansible/hosts -e target_group=$(SERVER)

sr:
	ssh -t $(SSH_HOST_ROOT)  'cd /root/ros2_ws && exec bash -l'

p:
	cd $(ROBOT_ROS_DIR) && exec bash

run:
	xhost +local:root
	echo "change" > $(ROBOT_ROS_DIR)/controller/change
	#sudo CONTAINER_NAME=dadou-$(SERVER)-container GUI=$(GUI) $(ROBOT_ROS_DIR)/conf/scripts/compose-up-local.sh
	sudo CONTAINER_NAME=dadou-$(SERVER)-container GUI=$(GUI) $(ROBOT_ROS_DIR)/conf/scripts/compose-up-local.sh
	xhost -local:root

build:
	xhost +local:root
	echo "change" > $(ROBOT_ROS_DIR)/controller/change
	#cat $(ROBOT_ROS_DIR)/conf/requirements.txt >  $(ROBOT_ROS_DIR)/conf/requirements-no-gui.txt
	#cat $(ROBOT_ROS_DIR)/conf/requirements-gpio.txt >>  $(ROBOT_ROS_DIR)/conf/requirements-no-gui.txt
	sudo CONTAINER_NAME=dadou-$(SERVER)-container GUI=$(GUI) MODE=build $(ROBOT_ROS_DIR)/conf/scripts/compose-up-local.sh
	xhost -local:root

in:
	sudo docker exec -it dadou-gl-container /bin/bash

l:
	tail -f $(LOCAL_TEST_LOG)

args:
	echo  "SERVER : $(SERVER) | ANSIBLE: $(ANSIBLE)"