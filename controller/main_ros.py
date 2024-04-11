import json
import logging
import logging.config

import rclpy
from rclpy.node import Node
from robot_interfaces.msg._string_time import StringTime
import tkinter as tk

from controller.control_config import config
from dadou_utils_ros.logging_conf import LoggingConf
from dadou_utils_ros.utils_static import AUDIO, FACE, ROBOT_LIGHTS, RELAYS, LEFT_EYE, NECK, RIGHT_EYE, LEFT_ARM, \
    RIGHT_ARM, ANIMATION, LOGGING_CONFIG_FILE, WHEELS, DURATION, LOGGING_FILE_NAME
from controller.gui.small_gui import SmallGui

PUBLISHER_LIST = [ANIMATION, AUDIO, FACE, ROBOT_LIGHTS, RELAYS, NECK ,LEFT_EYE, RIGHT_EYE, LEFT_ARM, RIGHT_ARM, WHEELS]


class Ros2TkinterApp(Node):
    def __init__(self):
        print(config[LOGGING_CONFIG_FILE])
        #logging.config.fileConfig(config[LOGGING_CONFIG_FILE], disable_existing_loggers=False)
        logging.config.dictConfig(LoggingConf.get(config[LOGGING_FILE_NAME], "controller"))
        super().__init__("controller_node")
        logging.info("start controller node")

        self.action_publishers = {}
        for p in PUBLISHER_LIST:
            self.action_publishers[p] = self.create_publisher(StringTime, p, 10)

        self.gui = SmallGui(self)

    def publish(self, animations_msg):
        if animations_msg and isinstance(animations_msg, dict):
            for k, v in animations_msg.items():
                if k in self.action_publishers:
                    logging.info("publish {} in {}".format(v, k))
                    msg = StringTime()
                    msg.msg = json.dumps(v)
                    if(DURATION in animations_msg):
                        msg.time = animations_msg[DURATION]
                    self.action_publishers[k].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ros_tk_app = Ros2TkinterApp()
    #rclpy.spin_once(ros_tk_app, timeout_sec=0.1)/home/dadou/Nextcloud/Didier/python/dadou_robot_ros/conf/ros2_dependencies/robot_interfaces

    while rclpy.ok():
        ros_tk_app.gui.update()
        rclpy.spin_once(ros_tk_app, timeout_sec=0)

    ros_tk_app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()