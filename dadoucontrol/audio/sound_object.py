import logging
from enum import Enum
from timeit import default_timer
from mutagen.mp3 import MP3
from pydub import AudioSegment, playback

from dadoucontrol.logic.time_utils import TimeUtils
from singleton import SingletonMeta


class State(Enum):
    PLAY = 1
    STOP = 2
    PAUSE = 3
    LOOP = 4


class SoundObject:

    starting_time = 0
    current_state = State.STOP

    def __init__(self, audio_folder=None, audio_name=None):
        self.audio_folder = audio_folder
        self.audio_name = audio_name
        if audio_name:
            logging.info('load sound ' + audio_folder + audio_name)
            self.audio_segment = AudioSegment.from_mp3(audio_folder+audio_name)
        self.play_obj = None
        self.pause_time = None

    def play(self):
        self.play_obj = playback._play_with_simpleaudio(self.audio_segment)
        self.current_state = State.PLAY

        """self.play_obj = simpleaudio.play_buffer(
            self.audio_segment.raw_data,
            num_channels=self.audio_segment.channels,
            bytes_per_sample=self.audio_segment.sample_width,
            sample_rate=self.audio_segment.frame_rate
        )"""

    def stop(self):
        self.play_obj.stop()
        self.current_state = State.STOP

    def pause(self, pause_time):
        self.pause_time = pause_time
        self.stop()
        self.current_state = State.PAUSE

    def resume(self):
        self.play_from(self.pause_time)

    def play_from(self, start_time):
        audio_split = self.audio_segment[start_time:]
        self.play_obj = playback._play_with_simpleaudio(audio_split)
        self.current_state = State.PLAY
        """self.play_obj = simpleaudio.play_buffer(
            audio_split.raw_data,
            num_channels=audio_split.channels,
            bytes_per_sample=audio_split.sample_width,
            sample_rate=audio_split.frame_rate
        )"""

    def get_duration(self):
        audio_path = self.audio_folder+self.audio_name
        audio = MP3(audio_path)
        logging.info('audio segment length : {}'.format(len(self.audio_segment)))
        return TimeUtils.formatted_time(audio.info.length)

    def is_playing(self):
        return self.play_obj.is_playing()

    def display_time(self):
        if self.is_playing():
            diff = default_timer() - self.starting_time
            return TimeUtils.formatted_time(diff)
        else:
            return 0

    def get_audio_data_display(self, width):

        max_frame = 4500000000
        #for i in range(int(audio.frame_count())):
        #    frame_value = int.from_bytes(audio.get_frame(i), 'little')
        #    if frame_value > max_frame:
        #        max_frame = abs(frame_value)

        nb_frames_pixel = self.audio_segment.frame_count()/width
        display_data = []
        for i in range(width):
            base = i*nb_frames_pixel
            total_for_pixel = 0
            for j in range(int(nb_frames_pixel)):
                total_for_pixel += int.from_bytes(self.audio_segment.get_frame(int(base+j)), 'little')
            average = total_for_pixel / nb_frames_pixel
            display_data.append(average/max_frame)
            #display_data.append(int.from_bytes(audio.get_frame(int(base)), 'little') / max_frame)

        return display_data




