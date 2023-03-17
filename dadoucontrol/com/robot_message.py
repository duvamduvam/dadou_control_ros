import logging
import traceback

from control_config import WS_CLIENT
from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.misc import Misc
from dadou_utils.com.ws_client import WsClient
from dadou_utils.utils_static import LORA, ANGLO, KEY, JOY, NECK, WHEEL_LEFT, WHEEL_RIGHT


class RobotMessage:

    def __init__(self, ws_client, device_manager: SerialDeviceManager):
        self.ws_client = ws_client
        self.lora = device_manager.get_device(LORA)

    def send(self, msg: dict):
        if self.lora and self.lora.exist():
            self.send_lora(msg)
        else:
            self.send_ws(msg)

    def send_sliders(self, msg: str):
        if len(msg) == 2:
            self.send({NECK: msg})
        if len(msg) == 4:
            left = Misc.mapping(int(msg[0:2]), 10, 99, -100, 100)
            right = Misc.mapping(int(msg[2:4]), 10, 99, -100, 100)
            self.send({WHEEL_LEFT: left, WHEEL_RIGHT: right})

    def send_ws(self, msg: dict):
        try:
            logging.info("send {}".format(msg))
            #TODO fix 1011 error
            self.ws_client.send(msg)
            #WsClient.async_send(msg, WS_CLIENT)
        except Exception as e:
            logging.error("can't send msg {}".format(e))
            traceback.print_exc()

    def send_lora(self, msg:dict):
        if ANGLO in msg:
            self.lora.send_msg('A'+msg[ANGLO])
        if KEY in msg:
            self.lora.send_msg('K'+msg[KEY])
        if JOY in msg:
            self.lora.send_msg('J'+msg[JOY])