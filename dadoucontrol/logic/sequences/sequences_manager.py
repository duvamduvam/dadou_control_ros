from dadoucontrol.audio.sound_object import SoundObject
from dadoucontrol.control_static import ControlStatic


class SequencesManagement:
    json_sequence = None
    audio_path = None
    audio_name = None
    music_frame = None
    audio_segment = SoundObject()

    def __init__(self, json_manager):
        self.json_manager = json_manager

    def load_sequence(self, sequence_name):
        self.json_sequence = self.json_manager.open_json(sequence_name, ControlStatic.SEQUENCES_DIRECTORY)
        audio_name = self.json_manager.get_attribut(self.json_sequence, ControlStatic.AUDIO_NAME_KEY)
        audio_folder = self.json_manager.get_attribut(self.json_sequence, ControlStatic.AUDIO_PATH_KEY)
        self.audio_segment = SoundObject(audio_folder, audio_name)

    def get_audio_name(self):
        return self.audio_name





