import os

from dadou_utils.misc import Misc


############## JSON FILES ##############
from dadou_utils.utils_static import LAPTOP_TYPE, RPI_TYPE

JSON_CONFIG = 'config.json'
JSON_EXPRESSIONS = 'expressions.json'
JSON_LIGHTS = 'lights.json'
JSON_LIGHTS_BASE = 'lights_base.json'
JSON_SPEECHS = 'speechs.json'

############### PATHS ###############

BASE_PATH = os.path.dirname(__file__)
AUDIO_DIRECTORY = '/audios/'
RPI_LOGGING_CONFIG_FILE = '/conf/logging-pi.conf'
LAPTOP_LOGGING_CONFIG_FILE = '/conf/logging-laptop.conf'
JSON_DIRECTORY = '/json/'
PLAYLIST_PATH = '/../json/playlists/'
SEQUENCES_DIRECTORY = '/sequences/'

############### KEYS ###############

AUDIO_NAME = 'audio_name'
AUDIO_PATH = 'audio_path'
GLOVE_LEFT = 'glove_left'
GLOVE_RIGHT = 'glove_right'
RANDOM_COLOR ='random_color'

############# COLORS ###############
#https://coolors.co/palettes/trending

PURPLE = '#5f0f40'
BORDEAUX = '#9a031e'
YELLOW = '#fb8b24'
ORANGE = '#e36414'
CYAN = '#0f4c5c'

############ FONTS #################

FONT1 = "Helvetica 18 italic bold" #None tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")
FONT2 = "Helvetica 15 italic bold" #None tkfont.Font(family='Helvetica', size=15, weight="bold", slant="italic")
FONT3 = "Helvetica 12 italic bold" #None tkfont.Font(family='Helvetica', size=12, weight="bold", slant="italic")

class ControlStatic:


    @staticmethod
    def get_logs_dir():
        system = Misc.get_system_type()
        if system == LAPTOP_TYPE:
            return LAPTOP_LOGGING_CONFIG_FILE
        elif system == RPI_TYPE:
            return RPI_LOGGING_CONFIG_FILE
