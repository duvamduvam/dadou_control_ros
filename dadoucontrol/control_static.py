from dadou_utils.misc import Misc
from dadou_utils.utils_static import UtilsStatic

class ControlStatic:

    JSON_CONFIG_NAME = 'config.json'

    CONFIG_FILE = 'config.json'
    EXPRESSIONS_FILE = 'expressions.json'
    LIGHTS_FILE = 'lights.json'
    LIGHTS_BASE_FILE = 'lights_base.json'
    RPI_LOGGING_CONFIG_FILE = '/conf/logging-pi.conf'
    LAPTOP_LOGGING_CONFIG_FILE = '/conf/logging-laptop.conf'
    JSON_DIRECTORY = "/json/"
    SEQUENCES_DIRECTORY = "/sequences/"

    GLOVE_LEFT = "glove_left"
    GLOVE_RIGHT = "glove_right"

    AUDIO_NAME_KEY = 'audio_name'
    AUDIO_PATH_KEY = 'audio_path'
    DEVICES_KEY = 'devices'
    NAME_KEY = 'name'
    PATH_KEY = 'path'
    PATHS_KEY = 'paths'
    SEQUENCES_DIRECTORY_KEY = 'sequences'
    WS_CLIENT_KEY = 'ws_client'

    @staticmethod
    def get_logs_dir():
        system = Misc.get_system_type()
        if system == UtilsStatic.LAPTOP_TYPE:
            return ControlStatic.LAPTOP_LOGGING_CONFIG_FILE
        elif system == UtilsStatic.RPI_TYPE:
            return ControlStatic.RPI_LOGGING_CONFIG_FILE
