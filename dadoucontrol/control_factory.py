import logging
import logging.config
import os

from dadou_utils.com.ws_client import WsClient
from dadou_utils.com.ws_server import WsMessage
from dadoucontrol.com.serial_device_manager import SerialDeviceManager

from dadoucontrol.audio.audio_navigation import AudioNav
from dadoucontrol.control_config import ControlConfig
from dadoucontrol.control_static import ControlStatic
from dadoucontrol.files.control_json_manager import ControlJsonManager
from dadou_utils.singleton import SingletonMeta
from dadou_utils.misc import Misc

#from dadou_utils.files.files_utils import FilesUtils
from dadoucontrol.logic.sequences.sequences_manager import SequencesManagement


class ControlFactory(metaclass=SingletonMeta):

    def __init__(self, base_path):
        self.VisualMouth = None
        self.VisualEye = None
        #current_path = os.path.dirname("main.py")
        logging.info("base path {}".format(base_path))
        # parent directory
        base_path = os.path.abspath(os.path.join(base_path, os.pardir))

        logging_file = base_path + ControlStatic.get_logs_dir()
        logging.config.fileConfig(logging_file, disable_existing_loggers=False)

        self.control_json_manager = ControlJsonManager(base_path, ControlStatic.JSON_DIRECTORY, ControlStatic.CONFIG_FILE)
        self.config = ControlConfig(self.control_json_manager, base_path)
        self.device_manager = SerialDeviceManager(self.control_json_manager)
        #self.files_utils = FilesUtils(self.config)
        self.ws_client = WsClient(ControlStatic.WS_CLIENT_URL)
        self.audio_nav = AudioNav()
        self.sequence_management = SequencesManagement(self.control_json_manager)

