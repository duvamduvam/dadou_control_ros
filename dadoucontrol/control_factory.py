import logging
import logging.config
import os

from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.com.ws_client import WsClient
from dadou_utils.utils_static import RPI_TYPE, LAPTOP_TYPE

from com.robot_message import RobotMessage

from audio.audio_navigation import AudioNav
from control_config import RPI_LOGGING_CONFIG_FILE, LAPTOP_LOGGING_CONFIG_FILE, \
    JSON_DIRECTORY, JSON_CONFIG, LOGGING_CONFIG_FILE, DEVICES, WS_CLIENT

from control_config import BASE_PATH
from files.control_json_manager import ControlJsonManager
from dadou_utils.singleton import SingletonMeta
from dadou_utils.misc import Misc

#from dadou_utils.files.files_utils import FilesUtils
from logic.sequences.sequences_manager import SequencesManagement


class ControlFactory(metaclass=SingletonMeta):

    def __init__(self):
        self.VisualMouth = None
        self.VisualEye = None

        logging.info("base path {}".format(BASE_PATH))

        print("config file {}".format(LOGGING_CONFIG_FILE))
        logging.config.fileConfig(LOGGING_CONFIG_FILE, disable_existing_loggers=False)

        self.control_json_manager = ControlJsonManager(JSON_DIRECTORY)
        self.device_manager = SerialDeviceManager(DEVICES)
        self.ws_client = WsClient(WS_CLIENT)
        self.message = RobotMessage(self.ws_client, self.device_manager)
        self.audio_nav = AudioNav()
        self.sequence_management = SequencesManagement(self.control_json_manager)
