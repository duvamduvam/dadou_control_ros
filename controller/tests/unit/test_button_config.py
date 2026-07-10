"""Cohérence des mappings boutons -> commandes.

Ros2TkinterApp.publish() ne publie que les clés présentes dans PUBLISHER_LIST
et ignore silencieusement les autres : ce test garantit qu'aucune commande de
BUTTONS_LAYOUT ne vise un topic inconnu (sauf commandes locales assumées).
"""

from dadou_utils_ros.utils_static import DURATION, NAME
from controller.control_static import CMD, PLAYLIST
from controller.buttons.button_config import BUTTONS_LAYOUT, KEYS_MAPPING
from controller.control_config import PUBLISHER_LIST

# Commandes traitées localement par la GUI, jamais publiées vers le robot.
LOCAL_COMMANDS = {PLAYLIST, DURATION, "mute"}
KNOWN_TARGETS = set(PUBLISHER_LIST) | LOCAL_COMMANDS


def all_buttons():
    for mode, layout in BUTTONS_LAYOUT.items():
        for button_id, entry in layout.items():
            yield mode, button_id, entry


def test_layout_entries_are_well_formed():
    for mode, button_id, entry in all_buttons():
        where = "{}[{}]".format(mode, button_id)
        assert entry == 0 or (isinstance(entry, dict) and NAME in entry and CMD in entry), \
            "{} : entrée invalide {!r}".format(where, entry)


def test_commands_target_known_topics():
    for mode, button_id, entry in all_buttons():
        if entry == 0:
            continue
        for command_key in entry[CMD]:
            assert command_key in KNOWN_TARGETS, \
                "{}[{}] '{}' : la clé '{}' n'est ni un topic publié ni une commande locale — " \
                "elle serait perdue silencieusement par publish()".format(
                    mode, button_id, entry[NAME], command_key)


def test_keyboard_mapping_points_to_existing_buttons():
    for key, button_id in KEYS_MAPPING.items():
        for mode, layout in BUTTONS_LAYOUT.items():
            assert button_id in layout, \
                "touche '{}' -> bouton {} absent du layout {}".format(key, button_id, mode)
