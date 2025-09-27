import copy
import logging
from dataclasses import dataclass
from typing import Optional

from dadou_utils_ros.singleton import SingletonMeta
from dadou_utils_ros.utils_static import AUDIO, ANGLO, BACKWARD, EYES, FORWARD, LEFT, LONG, MODE, MOUTH, NEXT, PLAYLIST, \
    RIGHT, \
    SHORT, WHEELS, NECK, ARMS, LEFT_ARM, RIGHT_ARM, DOWN, UP, NAME, CMD, KEY, FACE, ANIMATION, CONFIG, LIGHTS, STOP, \
    DEFAULT, SPEAK, INCLINO, RIGHT_EYE, LEFT_EYE, PLAYLIST_LIST, DURATION, TYPE, GLOVE, GAMEPAD, A, B, X, Y, L1, L2, R1, \
    R2, START, SELECT, LX, LY, RX, RY, XBOX
from controller.control_config import SINGLE_GLOVE_9DOF, SINGLE_GLOVE, \
    DUAL_GLOVE_9DOF_LEFT, DUAL_GLOVE_9DOF_RIGHT, DUAL_GLOVE_LEFT, DUAL_GLOVE_RIGHT, IHL, IML, IHR, IMR, IBR, MHL, MHR, \
    MML, MMR, MBL, MBR, AHL, AHR, AML, AMR, ABL, ABR, OHL, OHR, OML, OMR, OBL, OBR, IBL, config

BUTTONS = "buttons"
RELAY = "relay"
OCTAVER = "octaver"

INPUT_KEYS = [[IBL, MBL, ABL, OBL],
        [IML, MML, AML, OML],
        [IHL, MHL, AHL, OHL]]

GAMEPAD_KEYS = (A, B, X, Y, L1, L2, R1, R2, START, SELECT, MODE, UP, DOWN, LEFT, RIGHT)
JOYSTICK_KEYS = (LX, LY, RX, RY)


@dataclass(frozen=True)
class GamepadMapping:
    a_button: Optional[int] = None
    b_button: Optional[int] = None
    x_button: Optional[int] = None
    y_button: Optional[int] = None
    start_button: Optional[int] = None
    select_button: Optional[int] = None
    mode_button: Optional[int] = None
    l1_button: Optional[int] = None
    r1_button: Optional[int] = None
    l2_button: Optional[int] = None
    r2_button: Optional[int] = None
    lx_axis: Optional[int] = None
    ly_axis: Optional[int] = None
    rx_axis: Optional[int] = None
    ry_axis: Optional[int] = None
    l2_axis: Optional[int] = None
    r2_axis: Optional[int] = None
    invert_ly: bool = False
    invert_ry: bool = False


XBOX_MAPPING = GamepadMapping(
    a_button=0,
    b_button=1,
    x_button=3,
    y_button=4,
    start_button=11,
    select_button=10,
    mode_button=12,
    l1_button=6,
    r1_button=7,
    l2_button=8,
    r2_button=9,
    lx_axis=11,
    ly_axis=12,
    rx_axis=13,
    ry_axis=14,
    l2_axis=15,
    r2_axis=16,
    invert_ly=True,
    invert_ry=True,
)

GAMEPAD_MAPPING = {XBOX: XBOX_MAPPING}

KEYS_MAPPING = {'2': IHL, '1': IML, '12': IBL, '5': MHL, '4': MML, '3': MBL, '8': AHL, '7': AML, '6': ABL, '11': OHL, '10': OML, '9': OBL,
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
    OBR: {NAME: "reye up", CMD: {LEFT_EYE: UP}},

    # GAMEPAD
    A: {NAME: "ro vo", CMD: {RELAY: "pitched_voice", ANIMATION: "speak"}},
    B: {NAME: "vo off", CMD: {RELAY: "off"}},
    X: {NAME: "bug", CMD: {ANIMATION: {TYPE: "bug"}}},
    Y: {NAME: "klaxon", CMD: {RELAY: "klaxon"}},
    L1: {NAME: "neck left", CMD: {NECK: UP}},
    L2: {NAME: "larm down", CMD: {LEFT_ARM: UP}},
    R1: {NAME: "neck left", CMD: {NECK: DOWN}},
    R2: {NAME: "larm down", CMD: {LEFT_ARM: DOWN}},
    START: 0,
    SELECT: 0,
    MODE: 0,
    UP: {NAME: "wh for", CMD: {WHEELS: UP}},
    DOWN: {NAME: "wh back", CMD: {WHEELS: DOWN}},
    LEFT: {NAME: "wh left", CMD: {WHEELS: LEFT}},
    RIGHT: {NAME: "wh right", CMD: {WHEELS: RIGHT}}

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
        elif key in GAMEPAD_KEYS:
            button_cfg = BUTTONS_LAYOUT[DEFAULT].get(key)
            if isinstance(button_cfg, dict):
                return button_cfg.get(CMD)
            return button_cfg
        else:
            logging.error("key {} not in button layout".format(key))
