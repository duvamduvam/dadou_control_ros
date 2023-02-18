import logging
import tkinter as tk
from tkinter import font as tkfont, TOP, BOTH, ttk

from dadoucontrol.gui.windows.speech_window import SpeechFrame

from dadoucontrol.control_static import BORDEAUX, YELLOW, CYAN, ORANGE, PURPLE, FONT1, FONT3

from dadoucontrol.control_factory import ControlFactory

from dadoucontrol.gui.windows.expression_window import ExpressionFrame
from dadoucontrol.gui.windows.lights_window import LightsFrame
from dadoucontrol.gui.windows.remote_window import RemoteFrame
from dadoucontrol.gui.windows.section_window import SectionFrame

#https://www.hashbangcode.com/article/using-events-tkinter-canvas-elements-python
from gui.windows.frames.playlist_frame import PlaylistFrame
from gui.windows.keyboard_window import KeyboardFrame

class MainGui(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.option_add("*Font", FONT3)

        style = ttk.Style(self)
        # set ttk theme to "clam" which support the fieldbackground option
        style.theme_use("clam")

        tk.Grid.rowconfigure(self, 0, weight=1)
        tk.Grid.columnconfigure(self, 0, weight=1)

        self.EXIT = 'EXIT'
        self.EXPRESSION_FRAME = 'EXPRESSION'
        self.KEYBOARD_FRAME = 'KEYBOARD'
        self.PLAYLIST_FRAME = 'PLAYLIST'
        self.SEQUENCE_FRAME = 'SEQUENCE'
        self.LIGHTS_FRAME = 'LIGHTS'
        self.REMOTE_FRAME = 'REMOTE'
        self.CONFIG_FRAME = 'CONFIG'
        self.SPEECH_FRAME = 'SPEECH'

        #font1 = tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")

        self.geometry("1600x1024")
        self.attributes("-fullscreen", True)

        menu = tk.Frame(self, bg=ORANGE)
        menu.pack(fill='x', side=TOP)

        screen_width = self.winfo_screenwidth()

        tk.Button(menu, text='Exit', bg=PURPLE, font=FONT1, command=lambda: self.show_frame(self.EXIT)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
        tk.Button(menu, text='Keyboard', bg=BORDEAUX, font=FONT1, command=lambda: self.show_frame(self.KEYBOARD_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
        tk.Button(menu, text='Playlist', bg=YELLOW, font=FONT1, command=lambda: self.show_frame(self.PLAYLIST_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
        if screen_width > 1000:
            tk.Button(menu, text='Sequence', bg=ORANGE, font=FONT1, command=lambda: self.show_frame(self.SEQUENCE_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
            tk.Button(menu, text='Expression', bg=CYAN, font=FONT1, command=lambda: self.show_frame(self.EXPRESSION_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
            tk.Button(menu, text='Lights', bg=PURPLE, font=FONT1, command=lambda: self.show_frame(self.LIGHTS_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
            tk.Button(menu, text='Remote', bg=ORANGE, font=FONT1, command=lambda: self.show_frame(self.REMOTE_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
            tk.Button(menu, text='Speech', bg=BORDEAUX, font=FONT1, command=lambda: self.show_frame(self.SPEECH_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')
            tk.Button(menu, text='Config', bg=YELLOW, font=FONT1, command=lambda: self.show_frame(self.CONFIG_FRAME)).pack(ipadx=5, ipady=20, fill='x', expand=True, side='left')

        #self.main = tk.Frame(self, bg='yellow')


        self.main = KeyboardFrame(self)
        self.main.pack(fill=BOTH, expand=True, side=TOP)
        self.scheduler()

    def show_frame(self, frame_name):

        try:
            for child in self.main.winfo_children():
                child.destroy()
        except Exception as e:
            pass
        self.main.forget()
        self.main.destroy()

        if frame_name == self.KEYBOARD_FRAME:
            self.main = KeyboardFrame(self)
        elif frame_name == self.SEQUENCE_FRAME:
            self.main = SectionFrame(self)
        elif frame_name == self.PLAYLIST_FRAME:
            self.main = PlaylistFrame(self)
        elif frame_name == self.EXPRESSION_FRAME:
            self.main = ExpressionFrame(self)
        elif frame_name == self.LIGHTS_FRAME:
            self.main = LightsFrame(self)
        elif frame_name == self.REMOTE_FRAME:
            self.main = RemoteFrame(self)
        elif frame_name == self.SPEECH_FRAME:
            self.main = SpeechFrame(self)
        elif frame_name == self.CONFIG_FRAME:
            self.main = Config(self)
        elif frame_name == self.EXIT:
            quit()

        self.main.pack(fill=BOTH, expand=True, side=TOP)

    def scheduler(self):
        self.after(500, self.scheduler)
        ControlFactory().device_manager.update_devices()


class Config(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, bg=PURPLE, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
