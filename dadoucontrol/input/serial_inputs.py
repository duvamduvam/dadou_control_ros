import copy
import logging

from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.misc import Misc
from dadou_utils.utils_static import DEVICES, INPUT_KEY, BUTTON, SLIDERS, JOYSTICK, DEVICE, MSG, DIDIER, ALL, NECK, \
    WHEEL_LEFT, WHEEL_RIGHT, ROBOT, X, Y
from dadoucontrol.control_config import config
from dadou_utils.com.serial_devices_manager import SerialDeviceManager


class SerialInputs:

    def __init__(self):
        self.device_manager = SerialDeviceManager(config[DEVICES])
        self.serial_devices = {
            INPUT_KEY: self.device_manager.get_device_type(INPUT_KEY),
            BUTTON: self.device_manager.get_device_type(BUTTON),
            SLIDERS: self.device_manager.get_device_type(SLIDERS),
            JOYSTICK: self.device_manager.get_device_type(JOYSTICK)
        }
        self.input_key = None
        self.last_prompt = None

    def check_inputs(self):
        if self.check_input_type(serial_input=INPUT_KEY, add_to_list=False):
            self.input_key = self.last_prompt
        if self.check_input_type(serial_input=JOYSTICK) or self.check_input_type(serial_input=SLIDERS):
            return self.last_prompt

    def check_input_type(self, serial_input, add_to_list=True):
        if self.serial_devices[serial_input] and len(self.serial_devices[serial_input]) > 0:
            has_msg = False
            for device in self.serial_devices[serial_input]:
                msg = device.get_msg_separator()
                if msg:
                    if serial_input == SLIDERS:
                        msg = self.send_sliders(msg)
                    elif serial_input == JOYSTICK:
                        msg = self.send_joystick(msg)
                    logging.info("input {} : {}".format(serial_input, msg))
                    if add_to_list:
                        InputMessagesList().add_msg({DEVICE: device.name, serial_input: msg})
                    self.last_prompt = msg
                    has_msg = True
            return has_msg

    def get_key_msg(self):
        msg = copy.copy(self.input_key)
        self.input_key = None
        return msg

    def send_joystick(self, msg: str):
        if len(msg) == 4:
            x = Misc.mapping(int(msg[0:2]), 10, 99, -100, 100)
            y = Misc.mapping(int(msg[2:4]), 10, 99, -100, 100)
            return {X: x, Y: y}
        else:
            logging.error("wrong msg size {}".format(msg))

    def send_sliders(self, msg: str):
        if len(msg) == 2:
            return {NECK: msg}
        elif len(msg) == 4:
            left = Misc.mapping(int(msg[0:2]), 10, 99, -100, 100)
            right = Misc.mapping(int(msg[2:4]), 10, 99, -100, 100)
            return {WHEEL_LEFT: left, WHEEL_RIGHT: right}