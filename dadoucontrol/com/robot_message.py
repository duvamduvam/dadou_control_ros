import logging
import traceback

from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.utils_static import LORA, ANGLO, KEY, JOY


class RobotMessage:

    def __init__(self, ws_client, device_manager:SerialDeviceManager):
        self.ws_client = ws_client
        self.lora = device_manager.get_device(LORA)

    def send(self, msg:dict):
        if self.lora and self.lora.exist():
            self.send_lora(msg)
        else:
            self.send_ws(msg)

    def send_ws(self, msg:dict):
        try:
            logging.info("send {}".format(msg))
            self.ws_client.send(msg)
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