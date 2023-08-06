import copy

from dadou_utils.utils_static import AUDIO, ANGLO, BACKWARD, EYES, FORWARD, LEFT, LONG, MODE, MOUTH, NEXT, PLAYLIST, \
    RIGHT, \
    SHORT, WHEELS, NECK, ARMS, LEFT_ARM, RIGHT_ARM, DOWN, UP, NAME, CMD, KEY, FACE, ANIMATION, CONFIG, LIGHTS, STOP, \
    DEFAULT, SPEAK
from dadoucontrol.control_config import SINGLE_GLOVE_9DOF, MH, OH, OM, OB, MM, IB, IM, IH, AH, AM, AB, MB, SINGLE_GLOVE, \
    DUAL_GLOVE_9DOF_LEFT, DUAL_GLOVE_9DOF_RIGHT, DUAL_GLOVE_LEFT, DUAL_GLOVE_RIGHT

BUTTONS = "buttons"
RELAY = "relay"
OCTAVER = "octaver"

INPUT_KEYS = [[IB, MB, AB, OB],
        [IM, MM, AM, OM],
        [IH, MH, AH, OH]]

KEYS_MAPPING = {'a': IH, 'b': IM, 'c': IB, 'd': MH, 'e': MM, 'f': MB, 'g': AH, 'h': AM, 'i': AB, 'j': OH, 'k': OM, 'l': OB}

CONTROL_CONFIG = [[DEFAULT, PLAYLIST, "VIDE"], ["VIDE", "VIDE", "VIDE"], ["VIDE", "VIDE", "VIDE"]]
PLAYLIST_CONFIG = [["didier20_bis", "playlist3"]]

base = {
    IH: {NAME: "salut", CMD: {AUDIO: 'salut'}},
    IM: {NAME: "ca va", CMD: {AUDIO: 'ca-va'}},
    IB: {NAME: "sympa toi", CMD: {ANIMATION: 'crazy'}},
    MH: {NAME: "wh right", CMD: {WHEELS: RIGHT}},
    MM: {NAME: "wh left", CMD: {WHEELS: LEFT}},
    MB: {NAME: "ro vo", CMD: {RELAY: "pitched_voice", ANIMATION: "speak"}},
    AH: {NAME: "wh for", CMD: {WHEELS: FORWARD}},
    AM: {NAME: "wh back", CMD: {WHEELS: BACKWARD}},
    AB: {NAME: "no vo", CMD: {RELAY: "normal_voice"}},
    OH: {NAME: "mute", CMD: {"mute": "mute"}},
    OM: {NAME: "prout", CMD: {AUDIO: 'prout'}},
    OB: {NAME: "stop", CMD: {AUDIO: STOP}}
}

playlist = copy.deepcopy(base)
playlist[IB] = {NAME: "pl next", CMD: {PLAYLIST: NEXT}}

BUTTONS_LAYOUT = {
    PLAYLIST: playlist,
    DEFAULT: base
}

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
