import logging
import tkinter as tk
import time
from tkinter import TOP, BOTH, ttk, LEFT
from tkinter.messagebox import showinfo


from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.misc import Misc
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.utils_static import ORANGE, BORDEAUX, YELLOW, CYAN, FONT1, PURPLE, FONT3, BASE_PATH, PATHS, ICONS, \
    BUTTON_GRID, CMD, DEVICE, MSG, MODE, CONTROL, PLAYLIST, CONFIG, DEFAULT, FONT2, HOST_NAME, ERROR

from dadoucontrol.control_config import config, FONT_DROPDOWN, SINGLE_GLOVE
from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.windows.frames.widgets.Icons_widget import Icons_widget
from dadoucontrol.gui.windows.mod_window import ModWindow

from dadoucontrol.gui.windows.small.small_config import SmallConfig
from dadoucontrol.gui.windows.small.small_control import SmallControl
from dadoucontrol.gui.windows.small.small_playlist import SmallPlaylist
#from dadoucontrol.input.gampad import GamePad

MESSAGE_INPUT_TIMEOUT = 1000
MENU = [CONTROL, PLAYLIST, CONFIG]

class GamePadGui(tk.Tk):
    def __init__(self, tkMessageBox=None, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.geometry("480x320")
        if config[HOST_NAME] != '5401':
            self.wm_attributes('-type', 'splash')
        self.bind('<Escape>', lambda e: self.destroy())
        screen_width = self.winfo_screenwidth()
        if screen_width < 1000:
            self.attributes("-fullscreen", True)

        canvas = tk.Canvas(self, bg='white')
        canvas.pack(fill=BOTH)


