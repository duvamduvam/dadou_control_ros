import logging
import tkinter as tk
from tkinter import BOTH, TOP, LEFT, NW, END

from dadou_utils.misc import Misc
from dadou_utils.singleton import SingletonMeta
from dadou_utils.static_value import StaticValue

from gui.windows.frames.widgets.directory_tree_widget import DirectoryTreeWidget

from control_factory import ControlFactory

from control_static import CYAN, ORANGE
from dadou_utils.utils_static import FILES


class SpeechFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):

        tk.Frame.__init__(self, parent, bg=ORANGE, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)

        left_menu = tk.Frame(self, width=50, bg=CYAN)
        left_menu.pack(fill='y', ipadx=20, side='left')

        self.speech_data = ControlFactory().control_json_manager.get_speechs()

        self.speechs_var = tk.StringVar()
        self.speechs = tk.Listbox(left_menu, listvariable=self.speechs_var, height=15)
        self.speechs.pack(side=TOP)
        self.speechs.bind('<<ListboxSelect>>', self.load_speech)

        #TODO remove absolute path
        DirectoryTreeWidget(left_menu, "/home/dadou/Nextcloud/Didier/python/dadou_control/audios", FILES)

        self.speech_var = tk.StringVar()
        self.speech = tk.Listbox(self, listvariable=self.speech_var, height=15)
        self.speech.pack(side=TOP, anchor=NW, ipadx=10, ipady=10)
        self.speech.bind('<Double-Button-1>', self.load_speech)

        self.load_speechs()
        self.check_new_audio()

    def load_speech(self, e):
        w = e.widget
        if len(w.curselection()) > 0:
            index = int(w.curselection()[0])
            value = w.get(index)
            self.speech_var.set(self.speech_data[value])
        else:
            logging.error("no line selected")

    def check_new_audio(self):
        self.after(100, self.check_new_audio)
        value = StaticValue.get()
        if value:
            self.speech.insert(END, value)

    def load_speechs(self):
        results = []
        for speech in self.speech_data:
            results.append(speech)
        self.speechs_var.set(results)