from dadou_utils.misc import Misc


############## JSON FILES ##############
from dadou_utils.utils_static import LAPTOP_TYPE, RPI_TYPE

JSON_CONFIG = 'config.json'
JSON_EXPRESSIONS = 'expressions.json'
JSON_LIGHTS = 'lights.json'
JSON_LIGHTS_BASE = 'lights_base.json'

############### PATHS ###############

RPI_LOGGING_CONFIG_FILE = '/conf/logging-pi.conf'
LAPTOP_LOGGING_CONFIG_FILE = '/conf/logging-laptop.conf'
JSON_DIRECTORY = '/json/'
SEQUENCES_DIRECTORY = '/sequences/'

############### KEYS ###############

AUDIO_NAME = 'audio_name'
AUDIO_PATH = 'audio_path'
GLOVE_LEFT = 'glove_left'
GLOVE_RIGHT = 'glove_right'



class ControlStatic:


    @staticmethod
    def get_logs_dir():
        system = Misc.get_system_type()
        if system == LAPTOP_TYPE:
            return LAPTOP_LOGGING_CONFIG_FILE
        elif system == RPI_TYPE:
            return RPI_LOGGING_CONFIG_FILE
