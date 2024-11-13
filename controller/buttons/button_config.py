import copy
import logging

from dadou_utils_ros.singleton import SingletonMeta
from dadou_utils_ros.utils_static import AUDIO, ANGLO, BACKWARD, EYES, FORWARD, LEFT, LONG, MODE, MOUTH, NEXT, PLAYLIST, \
    RIGHT, \
    SHORT, WHEELS, NECK, ARMS, LEFT_ARM, RIGHT_ARM, DOWN, UP, NAME, CMD, KEY, FACE, ANIMATION, CONFIG, LIGHTS, STOP, \
    DEFAULT, SPEAK, INCLINO, RIGHT_EYE, LEFT_EYE, PLAYLIST_LIST, DURATION, TYPE
from controller.control_config import SINGLE_GLOVE_9DOF, SINGLE_GLOVE, \
    DUAL_GLOVE_9DOF_LEFT, DUAL_GLOVE_9DOF_RIGHT, DUAL_GLOVE_LEFT, DUAL_GLOVE_RIGHT, IHL, IML, IHR, IMR, IBR, MHL, MHR, \
    MML, MMR, MBL, MBR, AHL, AHR, AML, AMR, ABL, ABR, OHL, OHR, OML, OMR, OBL, OBR, IBL, config

BUTTONS = "buttons"
RELAY = "relay"
OCTAVER = "octaver"

INPUT_KEYS = [[IBL, MBL, ABL, OBL],
        [IML, MML, AML, OML],
        [IHL, MHL, AHL, OHL]]

KEYS_MAPPING = {'a': IHL, 'b': IML, 'c': IBL, 'd': MHL, 'e': MML, 'f': MBL, 'g': AHL, 'h': AML, 'i': ABL, 'j': OHL, 'k': OML, 'l': OBL,
                'm': IHR, 'n': IMR, 'o': IBR, 'p': MHR, 'q': MMR, 'r': MBR, 's': AHR, 't': AMR, 'u': ABR, 'v': OHR, 'w': OMR, 'x': OBR}

CONTROL_CONFIG = [[DEFAULT, PLAYLIST, "VIDE"], ["VIDE", "VIDE", "VIDE"], ["VIDE", "VIDE", "VIDE"]]
PLAYLIST_CONFIG = [config[PLAYLIST_LIST], ["pub1", "VIDE", "VIDE"]]

base = {
    IHL: {NAME: "vo off", CMD: {RELAY: "off"}},
    IML: {NAME: "ca va", CMD: {AUDIO: 'ca-va'}},
    IBL: {NAME: "sympa toi", CMD: {ANIMATION: 'crazy'}},
    MHL: {NAME: "wh right", CMD: {WHEELS: RIGHT}},
    MML: {NAME: "wh left", CMD: {WHEELS: LEFT}},
    MBL: {NAME: "ro vo", CMD: {RELAY: "pitched_voice", ANIMATION: "speak"}},
    AHL: {NAME: "wh for", CMD: {WHEELS: FORWARD}},
    AML: {NAME: "wh back", CMD: {WHEELS: BACKWARD}},
    ABL: {NAME: "klaxon", CMD: {RELAY: "klaxon"}},
    OHL: {NAME: "bug", CMD: {ANIMATION: {TYPE: "bug"}}},
    OML: {NAME: "bug1", CMD: {ANIMATION: "bug1",
        DURATION: 5000, AUDIO: "street/promotion-vie-eternelle.mp3"}},
    OBL: {NAME: STOP, CMD: {ANIMATION: STOP}},

    IHR: {NAME: "Inclino", CMD: {}},
    IMR: {NAME: "rarm down", CMD: {RIGHT_ARM: DOWN}},
    IBR: {NAME: "rarm up", CMD: {RIGHT_ARM: UP}},
    MHR: {NAME: "neck right", CMD: {NECK: DOWN}},
    MMR: {NAME: "larm down", CMD: {LEFT_ARM: DOWN}},
    MBR: {NAME: "larm up", CMD: {LEFT_ARM: UP}},
    AHR: {NAME: "neck left", CMD: {NECK: UP}},
    AMR: {NAME: "leye down", CMD: {RIGHT_EYE: DOWN}},
    ABR: {NAME: "leye up", CMD: {RIGHT_EYE: UP}},
    OHR: {NAME: "mute", CMD: {"mute": "mute"}},
    OMR: {NAME: "reye down", CMD: {LEFT_EYE: DOWN}},
    OBR: {NAME: "reye up", CMD: {LEFT_EYE: UP}}
}

playlist = copy.copy(base)
playlist[IBL] = {NAME: "pl next", CMD: {PLAYLIST: NEXT}}

BUTTONS_LAYOUT = {
    PLAYLIST: playlist,
    DEFAULT: base
}


class Buttons(metaclass=SingletonMeta):

    @staticmethod
    def get(type, key):
        if key[0] == "I":
            return {INCLINO: key[1:]}
        if key in KEYS_MAPPING:
            return BUTTONS_LAYOUT[type][KEYS_MAPPING[key]][CMD]
        else:
            logging.error("key {} not in button layout".format(key))

"""DUAL_GLOVE_9DOF_LEFT: {
    IH: {AUDIO: 'X'}, IM: {AUDIO: 'X'}, IB: {AUDIO: 'X'},
    MH: {LONG: {ANGLO: EYES}, SHORT: {AUDIO: 'X'}}, MM: {LONG: {ANGLO: LEFT_ARM}, SHORT: {AUDIO: 'X'}}, MB: {LONG: {ANGLO: EYES}, SHORT: {AUDIO: 'X'}},
    AH: {LONG: {WHEELS: LEFT}, SHORT: {AUDIO: 'X'}}, AM: {LONG: {WHEELS: RIGHT}, SHORT: {AUDIO: 'X'}}, AB: {AUDIO: 'X'},
    OH: {MODE: NEXT}, OM: {PLAYLIST: UP}, OB: {PLAYLIST: DOWN}
},
DUAL_GLOVE_9DOF_RIGHT: {
    IH: {RELAY: OCTAVER}, IM: {RELAY: OCTAVER}, IB: {PLAYLIST: NEXT},
    MH: {LONG: {ANGLO: WHEELS}, SHORT: {AUDIO: 'X'}}, MM: {LONG: {ANGLO: RIGHT_ARM}, SHORT: {AUDIO: 'X'}}, MB: {LONG: {ANGLO: MOUTH}, SHORT: {AUDIO: 'X'}},
    AH: {LONG: {WHEELS: LEFT}, SHORT: {AUDIO: 'X'}}, AM: {LONG: {WHEELS: RIGHT}, SHORT: {AUDIO: 'X'}}, AB: {AUDIO: 'X'},
    OH: {MODE: NEXT}, OM: {PLAYLIST: UP}, OB: {PLAYLIST: DOWN}
},
DUAL_GLOVE_LEFT: {
    IH: {RELAY: OCTAVER}, IM: {RELAY: OCTAVER}, IB: {PLAYLIST: NEXT},
    MH: {LONG: {WHEELS: FORWARD}, SHORT: {AUDIO: 'X'}}, MM: {LONG: {WHEELS: BACKWARD}, SHORT: {AUDIO: 'X'}}, MB: {LONG: {ANGLO: EYES}, SHORT: {AUDIO: 'X'}},
    AH: {LONG: {WHEELS: LEFT}, SHORT: {AUDIO: 'X'}}, AM: {LONG: {WHEELS: RIGHT}, SHORT: {AUDIO: 'X'}}, AB: {AUDIO: 'X'},
    OH: {MODE: NEXT}, OM: {PLAYLIST: UP}, OB: {PLAYLIST: DOWN}
}"""
