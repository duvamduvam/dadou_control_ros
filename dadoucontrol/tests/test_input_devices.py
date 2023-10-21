import logging
import time
import unittest

from buttons.gp_buttons import GPButtons
from control_config import config, BUTTONS_MAPPING
from control_factory import ControlFactory
from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.utils_static import DEVICES, INPUT_KEY, BUTTON


class TestInputDevices(unittest.TestCase):
    device_manager = SerialDeviceManager(config[DEVICES], [BUTTON])

    def test_buttons(self):

        current_time = TimeUtils.current_milli_time()

        while not TimeUtils.is_time(last_time=current_time, time_out=60000):

            self.device_manager.check_buttons(BUTTONS_MAPPING)
            while InputMessagesList().has_msg():
                print(InputMessagesList().pop_msg())
            time.sleep(0.01)

    def test_buttons2(self):

        device_manager = SerialDeviceManager(config[DEVICES], [BUTTON])
        input_buttons = GPButtons(device_manager)

        current_time = TimeUtils.current_milli_time()
        while not TimeUtils.is_time(last_time=current_time, time_out=60000):

            input_buttons.check_internal()
            input_buttons.check_external()

            messages = InputMessagesList().pop_dict()
            if messages:
                logging.info(messages)
                InputMessagesList().add_msg(messages)

            time.sleep(0.02)

if __name__ == '__main__':
    unittest.main()

