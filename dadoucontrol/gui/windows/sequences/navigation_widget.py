import logging
import time
import tkinter as tk
from enum import Enum

from dadoucontrol.audio.audio_navigation import State
from dadoucontrol.control_factory import ControlFactory


class NavigationWidget(tk.Frame):

    sequence_management = ControlFactory().sequence_management

    def __init__(self, parent, *args, **kwargs):
        self.parent = parent
        tk.Frame.__init__(self, parent, *args, **kwargs)

        tk.Button(self, text='play', command=self.play).grid(row=0, column=0, padx=10)
        tk.Button(self, text='pause').grid(row=0, column=1, padx=10)
        tk.Button(self, text='stop', command=self.stop).grid(row=0, column=2, padx=10)
        tk.Button(self, text='loop').grid(row=0, column=3, padx=10)

        tk.Label(self, text='current time').grid(row=1, column=0, columnspan=2)
        self.timer_txt = tk.StringVar()
        self.time_label = tk.Label(self, textvariable=self.timer_txt)
        self.time_label.grid(row=1, column=3, columnspan=2)

        self.pack(fill='x', side='top')

        self.update_timer()

    def play(self):
        self.sequence_management.audio_segment.play()
        #self.audio_nav.start()
        #self.play()

    def stop(self):
        self.sequence_management.audio_segment.stop()

    """def play(self):
        if self.audio_nav.is_playing():
            #self.after(200, self.play)
            #self.audio_nav.play()
            self.sequence_management.audio_segment.play()
    """

    def update_timer(self):
        self.after(200, self.update_timer)
        if self.sequence_management.audio_segment.current_state == State.PLAY:
            display_time = self.sequence_management.audio_segment.display_time()
            self.timer_txt.set(display_time)


