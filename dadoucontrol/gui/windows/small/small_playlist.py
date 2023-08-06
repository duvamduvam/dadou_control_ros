import asyncio
import logging
import os
import subprocess
import threading
import tkinter as tk
from tkinter import BOTH, TOP, filedialog, LEFT, X, Y, RIGHT, END
from playsound import playsound
import vlc

from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.files.files_utils import FilesUtils
from dadou_utils.utils_static import NAME, PLAYLISTS, AUDIO, STOP, INPUT_KEY, KEY, PLAYLIST_PLAY, BASE_PATH, BORDEAUX, \
    PLAYLIST_PATH, CYAN, AUDIOS_DIRECTORY, FONT1, FONT2, ANIMATION, PLAYLIST_STOP, SLIDERS, WHEELS, ORANGE, YELLOW, \
    DEVICE, MSG, CMD, PLAYLIST, NEXT, CONTROL, CONFIG, DEFAULT, FONT3
from dadou_utils.audios.sound_object import SoundObject
from dadoucontrol.buttons.button_config import KEYS_MAPPING, BUTTONS_LAYOUT, PLAYLIST_CONFIG

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.control_config import config, RESTART_APP, FONT_DROPDOWN, FONT_BUTTON

MENU = [CONTROL, PLAYLIST, CONFIG]
PLAYLIST_CMD_INTERVAL = 1000

class SmallPlaylist(tk.Frame):
    def __init__(self, parent, mode, *args, **kwargs):

        self.parent = parent
        self.mode = mode
        self.devices = None
        self.sliders = None
        self.playlist_last_cmd = 0

        self.control_json = ControlFactory().control_json_manager
        self.deviceManager = ControlFactory().device_manager
        self.input_key_play = config[PLAYLIST_PLAY]
        self.input_key_stop = config[PLAYLIST_STOP]
        self.input_key_restart_app = RESTART_APP

        self.current_pos = 0
        self.audio_process = None
        self.vlc_player = None

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
        self.main = tk.Frame(self, width=400, bg=config[BORDEAUX])
        self.main.pack(fill=BOTH, expand=True, side=RIGHT)
        self.files = tk.StringVar()
        self.files.set(FilesUtils.get_folder_files_name(config[BASE_PATH] + config[PLAYLIST_PATH]))

        self.playlist_data = None

        self.playlist_var = tk.StringVar()

        self.playlist_listbox = tk.Listbox(self.main, listvariable=self.playlist_var, selectbackground=config[BORDEAUX], width=22, font=config[FONT2])
        self.playlist_listbox.bind("<<ListboxSelect>>", self.playlist_click)
        self.playlist_listbox.pack(side=LEFT)

        button_width = 1
        #button_top_frame = tk.Frame(self.main)
        #button_top_frame.pack(side=TOP)
        self.send_button = tk.Button(self.main, bg=config[BORDEAUX], text='send', width=15, height=button_width, font=FONT_BUTTON, command=self.click_send)
        self.send_button.pack(side=TOP)
        #self.play_button = tk.Button(button_top_frame, bg=config[BORDEAUX], text='play', width=5, height=button_width, font=config[FONT3], command=self.click_play)
        #self.play_button.pack(side=RIGHT)

        self.stop_button = tk.Button(self.main, bg=config[YELLOW], text='stop', width=15, height=button_width, font=FONT_BUTTON, command=self.click_stop)
        self.stop_button.pack(side=TOP)

        self.up_button = tk.Button(self.main, text='up', bg=config[BORDEAUX], width=15, height=button_width, font=FONT_BUTTON, command=self.OnEntryUp)
        self.up_button.pack(side=TOP)

        self.down_button = tk.Button(self.main, bg=config[YELLOW], text='down', width=15, height=button_width, font=FONT_BUTTON, command=self.OnEntryDown)
        self.down_button.pack(side=TOP)

        self.get_playlist(self.mode)
        self.exec_input()

    def OnEntryDown(self):
        self.playlist_listbox.yview_scroll(1, "units")

    def OnEntryUp(self):
        self.playlist_listbox.yview_scroll(-1, "units")

    def click_add(self):
        filepath = filedialog.askopenfilename(initialdir=config[BASE_PATH] + config[AUDIOS_DIRECTORY],title="Open a Text File",
                                              filetypes=(("text    files", "*"), ("all files", "*.*")))
        self.files.append(filepath)

    def playlist_click(self, evt):
        self.current_pos = self.playlist_listbox.curselection()[0]

    @staticmethod
    async def async_playsound(audio_path):
        playsound(audio_path)

    def click_play(self):
        #audio = SoundObject(config[BASE_PATH] + '/..' + config[AUDIOS_DIRECTORY], self.playlist_listbox.get(self.playlist_listbox.curselection()[0]))
        #audio.play()
        playlist_num = self.playlist_listbox.curselection()[0]
        audio_params = list(self.playlist_data.values())[playlist_num]
        audio_path = config[BASE_PATH] + '/..' + config[AUDIOS_DIRECTORY] + audio_params[AUDIO]
        if os.path.isfile(audio_path):

            if self.vlc_player:
                self.vlc_player.stop()
            self.vlc_player = vlc.MediaPlayer(audio_path)
            self.vlc_player.play()

            self.next()
        else:
            logging.error("{} not available".format(audio_path))

    def click_send(self):
        playlist_num = self.playlist_listbox.curselection()[0]
        audio_params = list(self.playlist_data.values())[playlist_num]
        logging.info("send playlist number {} value {}".format(playlist_num, audio_params))
        self.next()
        ControlFactory().message.send_multi_ws(audio_params)

    def click_stop(self):
        ControlFactory().message.send_multi_ws({AUDIO: STOP, ANIMATION: False})
        if self.vlc_player:
            self.vlc_player.stop()

    def click_audio(self, evt):
        w = evt.widget
        if len(w.curselection()):
            self.current_pos = int(w.curselection()[0])
            self.playlist_listbox.select_set(self.current_pos)

            self.playlist_listbox.selection_clear(0, 'end')
            self.playlist_listbox.select_set(self.current_pos)

    def next(self):
        if self.current_pos < len(self.playlist_data.keys()):
            self.playlist_listbox.selection_clear(0, 'end')
            self.current_pos = self.current_pos+1
            self.playlist_listbox.select_set(self.current_pos)
            if self.current_pos > 4:
                self.OnEntryDown()

    def click_file(self, evt):
        w = evt.widget
        if len(w.curselection()):
            index = int(w.curselection()[0])
            value = w.get(index)
            logging.info('You selected item %d: "%s"' % (index, value))

            self.playlist_listbox.delete(0, "end")
            self.playlist = value
            self.get_playlist(self.playlist)

    def get_playlist(self, file):
        if file == DEFAULT:
            file = PLAYLIST_CONFIG[0][0]
        items = []
        self.playlist_data = self.control_json.open_json(PLAYLISTS + '/' + file)
        for key in self.playlist_data:
            items.append(key)

        self.playlist_var.set(items)
        self.current_pos = 0
        self.playlist_listbox.select_set(self.current_pos)

    def playlist_instructions(self, cmd):
        if TimeUtils.is_time(self.playlist_last_cmd, PLAYLIST_CMD_INTERVAL):
            if cmd == NEXT:
                self.click_send()
            self.playlist_last_cmd = TimeUtils.current_milli_time()

    def exec_input(self):
        self.after(100, self.exec_input)

        if InputMessagesList().has_mg():
            msg = InputMessagesList().pop_msg()

            if "glove" in msg[DEVICE]:
                value = BUTTONS_LAYOUT[PLAYLIST][KEYS_MAPPING[msg[MSG]]][CMD]
                logging.info("input msg {}".format(value))
                key_list = list(value.keys())
                self.parent.show_popup("{} : {}".format(key_list[0], value[key_list[0]]))
                if key_list[0] == PLAYLIST:
                    self.playlist_instructions(value[key_list[0]])
                else:
                    ControlFactory().message.send_multi_ws(value)
