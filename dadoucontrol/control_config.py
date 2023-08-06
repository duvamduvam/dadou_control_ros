import logging
import os
import socket

from dadou_utils.misc import Misc

from dadou_utils.utils_static import AUDIOS_DIRECTORY, BUTTON_GRID, EYES, RPI_TYPE, LAPTOP_TYPE, NAME, SERIAL_ID, \
    MSG_SIZE, TYPE, PATH, SEQUENCES, \
    MOUTH, VISUALS, VISUALS, LAPTOP_TYPE, RPI_TYPE, BASE_PATH, WS_CLIENT, JSON_EXPRESSIONS, JSON_LIGHTS, JSON_DIRECTORY, \
    SEQUENCES_DIRECTORY, AUDIO_NAME, AUDIO_PATH, DEVICES, PATHS, LOGGING_CONFIG_FILE, JSON_CONFIG, JSON_LIGHTS_BASE, \
    JSON_SPEECHS, PLAYLIST_PATH, GLOVE_LEFT, GLOVE_RIGHT, PROJECT_LIGHTS_DIRECTORY, \
    PLAYLIST_PLAY, PLAYLIST_STOP, PURPLE, BORDEAUX, YELLOW, ORANGE, CYAN, FONT1, FONT2, FONT3, SYSTEM, \
    RPI_LOGGING_CONFIG_FILE, LAPTOP_LOGGING_CONFIG_FILE, JSON_LIGHTS_METHODS, WS_CLIENTS, WS_PORT, LOGGING_FILE_NAME, \
    BAUD_RATE, ICONS, CONFIG, LOG_FILE, HOST_NAME

config = {}

DUAL_GLOVE_9DOF_LEFT, DUAL_GLOVE_9DOF_RIGHT, DUAL_GLOVE_LEFT, DUAL_GLOVE_RIGHT, SINGLE_GLOVE_9DOF, SINGLE_GLOVE, DUAL_GLOVE, CONFIG = \
    "2 GL 9", "2 GR 9", "2 GL", "2 GR", "1 G 9", "1 G", "2 G", CONFIG
IH, IM, IB, MH, MM, MB, AH, AM, AB, OH, OM, OB = 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12

config[BASE_PATH] = os.getcwd()
config[HOST_NAME] = socket.gethostname()

config[WS_PORT] = 4421
#config[WS_CLIENTS] = {'robot': '192.168.1.200', 'sceno': '192.168.1.220', 'harddrive': '192.168.1.230'}
config[WS_CLIENTS] = {'robot': 'didier.local', 'sceno': 'sceno.local', 'harddrive': 'disk.local'}
#config[WS_CLIENT] = {'sceno': 'ws://192.168.1.220:4421', 'sceno': 'ws://192.168.1.220:4421'}
############## JSON FILES ##############

config[LOGGING_FILE_NAME] = "logs/control.log"

config[JSON_CONFIG] = 'control_config.json'
config[JSON_EXPRESSIONS] = 'expressions.json'
config[JSON_LIGHTS] = 'lights_base.json'
config[JSON_LIGHTS_BASE] = 'lights_base.json'
config[JSON_LIGHTS_METHODS] = 'lights_methods.json'
config[JSON_SPEECHS] = 'speechs.json'

############### PATHS ###############

config[BASE_PATH] = os.path.dirname(__file__)
config[AUDIOS_DIRECTORY] = '/audios/'
config[RPI_LOGGING_CONFIG_FILE] = '/../conf/logging-pi.conf'
config[LAPTOP_LOGGING_CONFIG_FILE] = '/../conf/logging-laptop.conf'
config[JSON_DIRECTORY] = '/../json/'
config[PLAYLIST_PATH] = '/../json/playlists/'
config[SEQUENCES_DIRECTORY] = '/sequences/'
config[PROJECT_LIGHTS_DIRECTORY] = '/projects_lights/'
############# COLORS ###############
#https://coolors.co/palettes/trending

config[PURPLE] = '#5f0f40'
config[BORDEAUX] = '#9a031e'
config[YELLOW] = '#fb8b24'
config[ORANGE] = '#e36414'
config[CYAN] = '#0f4c5c'

############ FONTS #################

config[FONT1] = "Helvetica 30 italic bold" #None tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")
config[FONT2] = "Helvetica 17 italic bold" #None tkfont.Font(family='Helvetica', size=15, weight="bold", slant="italic")
config[FONT3] = "Helvetica 15 italic bold" #None tkfont.Font(family='Helvetica', size=12, weight="bold", slant="italic")

FONT_DROPDOWN = "Helvetica 30 italic bold"
FONT_BUTTON = "Helvetica 34 italic bold"

config[BUTTON_GRID] = "Helvetica 31 italic bold" #None tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")

config[PLAYLIST_PLAY] = ['c', 'D']
config[PLAYLIST_STOP] = ['j', 'H']
RESTART_APP = ['I']
config[DEVICES] = [
        {
            NAME: 'buttons',
            SERIAL_ID: 'usb-1a86_USB_Serial-if00-port0',
            MSG_SIZE: 0,
            TYPE: "input_key",
            BAUD_RATE: 115200
        },
        {
            NAME: GLOVE_LEFT,
            SERIAL_ID: 'usb-Raspberry_Pi_Pico_E4627857633F1933-if00',
            MSG_SIZE: 0,
            TYPE: "input_key",
            BAUD_RATE: 115200
        },
        {
            NAME: "glove_right",
            SERIAL_ID: "tochange",
            "msg_size": 0,
            TYPE: "input_key",
            BAUD_RATE: 115200
        },
        {
            NAME: "joy",
            SERIAL_ID: "usb-FTDI_FT232R_USB_UART_AD0KBT1R-if00-port0",
            "msg_size": 6,
            TYPE: "joy",
            BAUD_RATE: 115200
        },
        {
            NAME: "sliders",
            SERIAL_ID: "usb-Raspberry_Pi_Pico_E66138935F269628-if00",
            "msg_size": 6,
            TYPE: "sliders",
            BAUD_RATE: 9600
        }
    ]

config[PATHS] = {
        VISUALS: "/../visuals/",
        EYES: "/../visuals/eye/",
        MOUTH: "/../visuals/mouth/",
        SEQUENCES: "/../json/sequences/",
        ICONS: "/../visuals/icons/"
    }

config[SYSTEM] = Misc.get_system_type()

if config[SYSTEM] in RPI_TYPE:
    config[LOGGING_CONFIG_FILE] = config[BASE_PATH] + config[RPI_LOGGING_CONFIG_FILE]
elif config[SYSTEM] == LAPTOP_TYPE:
    config[LOGGING_CONFIG_FILE] = config[BASE_PATH] + config[LAPTOP_LOGGING_CONFIG_FILE]
else:
    logging.error("can't find system type {}".format(config[SYSTEM]))

