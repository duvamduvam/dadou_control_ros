import asyncio
import logging
import traceback

from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.misc import Misc
from dadou_utils.utils_static import LORA, ANGLO, KEY, JOY, NECK, WHEEL_LEFT, WHEEL_RIGHT


class RobotMessage:

    #TODO improve event list / make good loop thread design
    event_loop = asyncio.new_event_loop()

    def __init__(self, ws_clients, device_manager: SerialDeviceManager):
        self.ws_clients = ws_clients
        self.lora = device_manager.get_device(LORA)

    def send(self, msg: dict):
        if self.lora and self.lora.exist():
            self.send_lora(msg)
        else:
            self.send_multi_ws(msg)

    def send_sliders(self, msg: str):
        if len(msg) == 2:
            self.send({NECK: msg})
        if len(msg) == 4:
            left = Misc.mapping(int(msg[0:2]), 10, 99, -100, 100)
            right = Misc.mapping(int(msg[2:4]), 10, 99, -100, 100)
            self.send({WHEEL_LEFT: left, WHEEL_RIGHT: right})

    def send_multi_ws(self, msg: dict):
        for ws_client in self.ws_clients:
                ws_client.send(msg)
                   # loop.create_task(foo())
                    #asyncio.run(self.send_ws(msg, ws_client))
                    #self.event_loop.create_task(self.send_ws(msg, ws_client))
                    #Ws_client.async_send(msg)

    @staticmethod
    async def send_ws(msg, ws_client):
        try:
            logging.info("send {} to {}".format(msg, ws_client.name))
            ws_client.send(msg)
        except Exception as e:
            logging.error("{} can't send msg {}".format(ws_client.name, e))
            traceback.print_exc()

    def send_lora(self, msg:dict):
        if ANGLO in msg:
            self.lora.send_msg('A'+msg[ANGLO])
        if KEY in msg:
            self.lora.send_msg('K'+msg[KEY])
        if JOY in msg:
            self.lora.send_msg('J'+msg[JOY])