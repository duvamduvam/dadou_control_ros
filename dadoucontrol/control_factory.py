import logging
import logging.config

from dadou_utils.utils_static import BASE_PATH, LOGGING_CONFIG_FILE, DEVICES, JSON_DIRECTORY, WS_CLIENT

from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.com.ws_client import WsClient
from dadou_utils.singleton import SingletonMeta

from dadoucontrol.audio.audio_navigation import AudioNav
from dadoucontrol.com.robot_message import RobotMessage
from dadoucontrol.control_config import config
from dadoucontrol.files.control_json_manager import ControlJsonManager
from dadoucontrol.logic.sequences.sequences_manager import SequencesManagement

from dadou_utils.utils_static import WS_CLIENTS, WS_PORT


class ControlFactory(metaclass=SingletonMeta):

    def __init__(self):
        self.VisualMouth = None
        self.VisualEye = None

        print("config file {}".format(config[LOGGING_CONFIG_FILE]))
        logging.config.fileConfig(config[LOGGING_CONFIG_FILE], disable_existing_loggers=False)

        self.control_json_manager = ControlJsonManager()
        self.device_manager = SerialDeviceManager(config[DEVICES])

        self.ws_clients = []
        for key, value in config[WS_CLIENTS].items():
            self.ws_clients.append(WsClient(value, config[WS_PORT], key))

        self.message = RobotMessage(self.ws_clients, self.device_manager)
        self.audio_nav = AudioNav()
        self.sequence_management = SequencesManagement(self.control_json_manager)
