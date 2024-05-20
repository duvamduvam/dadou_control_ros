#!/usr/bin/env python3
import json
import logging
import logging.config
import time

import board
import rclpy
from rclpy.node import Node
from robot_interfaces.msg._string_time import StringTime

from controller.buttons.button_config import Buttons
from controller.circuit_py.glove_keys import GloveKeys
from controller.circuit_py.vibrator import Vibrator
from controller.control_config import PUBLISHER_LIST
from controller.input.serial_inputs import SerialInputs
from controller.nodes.abstract_control_node import AbstractControllerNode
from dadou_utils_ros.logging_conf import LoggingConf
from dadou_utils_ros.utils_static import NECK, HEAD_PWM_NB, DEFAULT_POS, I2C_ENABLED, PWM_CHANNELS_ENABLED, \
    SERVOS, MAX_POS, SERVO, NAME, LOGGING_FILE_NAME, DURATION, CONTROL, DEFAULT
from robot.actions.servo import Servo
from robot.robot_config import config


class ControlNoGuiNode(AbstractControllerNode):
    def __init__(self):

        super().__init__("control_no_gui")
        logging.info("Starting nogui node")

        self.action_publishers = {}
        for p in PUBLISHER_LIST:
            self.action_publishers[p] = self.create_publisher(StringTime, p, 10)

        self.glove_keys = GloveKeys(((Buttons.get(DEFAULT, "x"), Buttons.get(DEFAULT, "w"), Buttons.get(DEFAULT, "v")),
                                (Buttons.get(DEFAULT, "r"), Buttons.get(DEFAULT, "q"), Buttons.get(DEFAULT, "p")),
                                (Buttons.get(DEFAULT, "o"), Buttons.get(DEFAULT, "n"), Buttons.get(DEFAULT, "m")),
                                (Buttons.get(DEFAULT, "u"), Buttons.get(DEFAULT, "t"), Buttons.get(DEFAULT, "s"))),
                             (board.GP10, board.GP11, board.GP12),
                             (board.GP6, board.GP7, board.GP8, board.GP9))

        self.vibrator = Vibrator(board.GP14)

        SDA = board.GP2
        SCL = board.GP3
        i2c = busio.I2C(SCL, SDA)  # uses board.SCL and board.SDA
        sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            cmd = self.glove_keys.check()
            if len(cmd) > 0:
                self.publish(cmd)
                self.vibrator.click()
            self.vibrator.process()
        except Exception as e:
            logging.error("error : {}".format(e))

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
