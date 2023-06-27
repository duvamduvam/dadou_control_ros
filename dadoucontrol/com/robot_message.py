import asyncio
import logging
import traceback

from dadou_utils.com.message import Message
from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.misc import Misc
from dadou_utils.utils_static import LORA, ANGLO, KEY, JOY, NECK, WHEEL_LEFT, WHEEL_RIGHT


class RobotMessage(Message):
    #TODO improve event list / make good loop thread design

    def send_sliders(self, msg: str):
        if len(msg) == 2:
            self.send({NECK: msg})
        if len(msg) == 4:
            left = Misc.mapping(int(msg[0:2]), 10, 99, -100, 100)
            right = Misc.mapping(int(msg[2:4]), 10, 99, -100, 100)
            self.send({WHEEL_LEFT: left, WHEEL_RIGHT: right})
