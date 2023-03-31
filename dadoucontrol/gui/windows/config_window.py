import logging
from tkinter import BOTH, TOP, X, HORIZONTAL, LEFT, RIGHT

import tkinter as tk

from dadou_utils.utils_static import BORDEAUX, FONT1

from control_config import config


class ConfigWindow(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        #self.main = tk.Frame.__init__(self, parent, bg=PURPLE, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
        self.exit_button = tk.Button(self, text='Exit', bg=config[BORDEAUX], font=config[FONT1],
                  command=exit)
        self.exit_button.pack(fill=X, side=TOP)

        volume_frame = tk.Frame(self)
        volume_label = tk.Label(volume_frame, text="Volume")
        volume_label.pack(fill=X, side=LEFT)
        self.volume_scale = tk.Scale(volume_frame, from_=0, to=100, length=500, resolution=1, command=self.update_volume(), orient=HORIZONTAL)
        self.volume_scale.pack(fill=X, side=LEFT)
        volume_frame.pack(fill=X, side=TOP)

    def update_volume(self):
        logging.info("volume changed {}".format(self.volume_scale.get()))

    def exit(self):
        exit()