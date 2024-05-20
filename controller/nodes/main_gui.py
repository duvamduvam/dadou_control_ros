import json
import logging
import logging.config

import rclpy
from rclpy.node import Node
from robot_interfaces.msg._string_time import StringTime
import tkinter as tk

from controller.control_config import config, PUBLISHER_LIST
from dadou_utils_ros.logging_conf import LoggingConf
from dadou_utils_ros.utils_static import AUDIO, FACE, ROBOT_LIGHTS, RELAY, LEFT_EYE, NECK, RIGHT_EYE, LEFT_ARM, \
    RIGHT_ARM, ANIMATION, LOGGING_CONFIG_FILE, WHEELS, DURATION, LOGGING_FILE_NAME, WHEEL_LEFT, WHEEL_RIGHT
from controller.gui.small_gui import SmallGui


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

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.gui.update()

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
    try:
        rclpy.init(args=args)
        ros_tk_app = Ros2TkinterApp()
        rclpy.spin(ros_tk_app)
        rclpy.shutdown()
    except Exception as e:
        logging.error(e, exc_info=True)

if __name__ == '__main__':
    main()