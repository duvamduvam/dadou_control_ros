import logging

from dadou_utils.audios.sound_object import SoundObject
from dadou_utils.utils_static import NAME, PATH, LENGTH, KEYS, AUDIOS

from control_config import AUDIO_DIRECTORY
from dadou_utils.utils_static import AUDIO_PATH


class SequencesManagement:
    audio_segment = None
    json_sequence = None
    audio_path = None
    audio_name = None
    music_frame = None

    def __init__(self, json_manager):
        self.json_manager = json_manager

    def load_audio(self, sequence_name):
        self.json_sequence = self.json_manager.get_sequence(sequence_name)
        if AUDIOS in self.json_sequence.keys():
            audios = self.json_sequence[AUDIOS]
            #audio_folder = self.json_manager.get_attribut(self.json_sequence, AUDIO_PATH)
            #if self.audio_segment is not None and hasattr(self.audio_segment, 'audio_segment'):
            #    self.audio_segment.stop()
            #TODO improve path and multi audios
            self.audio_segment = SoundObject(AUDIO_DIRECTORY, audios[0][1])
        else:
            logging.error("no audio in sequence {}".format(sequence_name))

    def get_audio_name(self):
        return self.audio_name

    def save_expression(self, left_eyes, right_eyes, mouths):
        logging.info('truc')

    def save_sequence(self, name, length, keys, audio_name, audio_path, lights, faces, necks, wheels):
        sequence = {LENGTH: length,
                    KEYS : keys,
                    'audio_name': audio_name,
                    'audio_path': audio_path,
                    'faces': faces,
                    'lights': lights,
                    'necks': necks,
                    'wheels': wheels}
        if not name.endswith('json'):
            name = name+".json"
        self.json_manager.save_file(sequence, name, ControlStatic.SEQUENCES_DIRECTORY)

    def get_parts(self, part_name):
        if part_name in self.json_sequence:
            return self.json_sequence[part_name]
        else:
            logging.error("key {} not present in sequences".format(part_name))
            return {}


