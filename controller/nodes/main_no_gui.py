#!/usr/bin/env python3
import json
import logging
import logging.config
import time

import rclpy
from rclpy.node import Node
from robot_interfaces.msg._string_time import StringTime

from controller.buttons.button_config import Buttons
from controller.control_config import PUBLISHER_LIST
from controller.input.serial_inputs import SerialInputs
from dadou_utils_ros.logging_conf import LoggingConf
from dadou_utils_ros.utils_static import NECK, HEAD_PWM_NB, DEFAULT_POS, I2C_ENABLED, PWM_CHANNELS_ENABLED, \
    SERVOS, MAX_POS, SERVO, NAME, LOGGING_FILE_NAME, DURATION, CONTROL, DEFAULT
from robot.actions.servo import Servo
from robot.robot_config import config


class ControlNoGuiNode(Node):
    def __init__(self):

        super().__init__("control_no_gui")
        logging.info("Starting nogui node")

        self.serial_inputs = SerialInputs(self)

        self.action_publishers = {}
        for p in PUBLISHER_LIST:
            self.action_publishers[p] = self.create_publisher(StringTime, p, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

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

    def timer_callback(self):
        serial_messages = self.parent.serial_inputs.get_key_msg()
        if serial_messages and len(serial_messages) > 0:
            for msg in serial_messages:
                value = Buttons.get(DEFAULT, msg)  # BUTTONS_LAYOUT[self.mode][KEYS_MAPPING[msg[MSG]]][CMD]
                if not value:
                    return

                logging.info("input msg {}".format(value))
                key_list = list(value.keys())
                logging.info("{} : {}".format(key_list[0], value[key_list[0]]))
                self.publish(value)
def main(args=None):
    rclpy.init(args=args)
    node = ControlNoGuiNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        logging.error(e, exc_info=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
