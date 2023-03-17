import logging
import os

from dadou_utils.utils_static import EYES, RPI_TYPE, LAPTOP_TYPE, NAME, SERIAL_ID, MSG_SIZE, TYPE, PATH, SEQUENCES, \
    MOUTH, VISUALS, VISUALS

from misc import Misc

BASE_PATH = os.getcwd()

WS_CLIENT = 'ws://192.168.1.200:4421'

############## JSON FILES ##############
from dadou_utils.utils_static import LAPTOP_TYPE, RPI_TYPE

JSON_CONFIG = 'control_config.json'
JSON_EXPRESSIONS = 'expressions.json'
JSON_LIGHTS = 'lights.json'
JSON_LIGHTS_BASE = 'lights_base.json'
JSON_SPEECHS = 'speechs.json'

############### PATHS ###############

BASE_PATH = os.path.dirname(__file__)
AUDIO_DIRECTORY = '/audios/'
RPI_LOGGING_CONFIG_FILE = '/conf/logging-pi.conf'
LAPTOP_LOGGING_CONFIG_FILE = '/../conf/logging-laptop.conf'
JSON_DIRECTORY = '/../json/'
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

PLAYLIST_PLAY = "c"
PLAYLIST_STOP = "j"
DEVICES = [
    {
        NAME: GLOVE_LEFT,
        SERIAL_ID: 'usb-Raspberry_Pi_Pico_E4683818DF310B23-if00',
        MSG_SIZE: 0,
        TYPE: "input_key"
    },
    {
        NAME: "glove_right",
        SERIAL_ID: "tochange",
        "msg_size": 0,
        TYPE: "input_key"
    },
    {
        NAME: "lora",
        SERIAL_ID: "usb-1a86_USB2.0-Serial-if00-port0",
        "msg_size": 0,
        TYPE: "lora"
    },
    {
        NAME: "joy",
        SERIAL_ID: "usb-FTDI_FT232R_USB_UART_AD0KBT1R-if00-port0",
        "msg_size": 6,
        TYPE: "joy"
    },
    {
        NAME: "sliders",
        SERIAL_ID: "usb-Raspberry_Pi_Pico_DE61B868C73F3036-if00",
        "msg_size": 6,
        TYPE: "sliders"
    }
]

PATHS = {
        VISUALS: "/../visuals/",
        EYES: "/../visuals/eye/",
        MOUTH: "/../visuals/mouth/",
        SEQUENCES: "/../json/sequences/"
    }


system = Misc.get_system_type()
if system in RPI_TYPE:
    LOGGING_CONFIG_FILE = BASE_PATH + RPI_LOGGING_CONFIG_FILE
elif system == LAPTOP_TYPE:
    LOGGING_CONFIG_FILE = BASE_PATH + LAPTOP_LOGGING_CONFIG_FILE
else:
    logging.error("can't find system type {}".format(system))
