SHELL := /bin/bash # Use bash syntax

ARG := $(word 3, $(MAKECMDGOALS) )

d:
	ansible-playbook /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/deploy-$(server)-pios.yml -i /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/hosts

dt:
	ansible-playbook /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/deploy-$(server)-test-pios.yml -vv -i /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/hosts

i:
	ansible-playbook /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/install-$(server)-pios.yml -i /home/dadou/Nextcloud/Didier/python/dadou_utils_ros/anisble/hosts

s:
	ssh $(server)

p:
	cd ~/Nextcloud/Didier/python/dadou_control_ros/ && exec bash

run:
	~/Nextcloud/Didier/python/dadou_control_ros/conf/scripts/compose-up-local.sh
