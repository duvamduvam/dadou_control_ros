import logging
import logging.config
import os

from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadou_utils.com.ws_client import WsClient
from dadou_utils.utils_static import RPI_TYPE, LAPTOP_TYPE, WS_CLIENT, DEVICES

from com.robot_message import RobotMessage

from audio.audio_navigation import AudioNav
from control_config import ControlConfig
from control_static import RPI_LOGGING_CONFIG_FILE, LAPTOP_LOGGING_CONFIG_FILE, \
    JSON_DIRECTORY, JSON_CONFIG
from files.control_json_manager import ControlJsonManager
from dadou_utils.singleton import SingletonMeta
from dadou_utils.misc import Misc

#from dadou_utils.files.files_utils import FilesUtils
from logic.sequences.sequences_manager import SequencesManagement


class ControlFactory(metaclass=SingletonMeta):

    def __init__(self, base_path):
        self.VisualMouth = None
        self.VisualEye = None
        #current_path = os.path.dirname("main.py")
        logging.info("base path {}".format(base_path))
        # parent directory
        self.base_path = os.path.abspath(os.path.join(base_path, os.pardir))

        system = Misc.get_system_type()
        if system == RPI_TYPE:
            logging_file = self.base_path + RPI_LOGGING_CONFIG_FILE
        elif system == LAPTOP_TYPE:
            logging_file = self.base_path + LAPTOP_LOGGING_CONFIG_FILE
        else:
            logging.error("can't find system type {}".format(system))


        logging.config.fileConfig(logging_file, disable_existing_loggers=False)

        self.control_json_manager = ControlJsonManager(self.base_path, JSON_DIRECTORY, JSON_CONFIG)
        self.config = ControlConfig(self.control_json_manager, self.base_path)
        self.device_manager = SerialDeviceManager(self.control_json_manager.get_config_item(DEVICES))
        self.ws_client = WsClient(self.control_json_manager.get_config_item(WS_CLIENT))
        self.message = RobotMessage(self.ws_client, self.device_manager)
        self.audio_nav = AudioNav()
        self.sequence_management = SequencesManagement(self.config, self.control_json_manager)
