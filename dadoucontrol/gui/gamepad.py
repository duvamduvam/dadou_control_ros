import logging
import os
import sys
import tkinter as tk
import time
from tkinter import TOP, BOTH, ttk, LEFT
from tkinter.messagebox import showinfo

from dadou_utils.com.serial_devices_manager import SerialDeviceManager
from dadoucontrol.buttons.gp_buttons import GPButtons
from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.misc import Misc
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.utils_static import ORANGE, BORDEAUX, YELLOW, CYAN, FONT1, PURPLE, FONT3, BASE_PATH, PATHS, ICONS, \
    BUTTON_GRID, CMD, DEVICE, MSG, MODE, CONTROL, PLAYLIST, CONFIG, DEFAULT, FONT2, HOST_NAME, ERROR, UP, DOWN, RIGHT, \
    Y, X, A, B, DEVICES, BUTTON, BL, BR, SELECT

from dadoucontrol.control_config import config, FONT_DROPDOWN, SINGLE_GLOVE, BUTTONS_MAPPING, SELECT_MODE
from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.windows.frames.widgets.icons_widget import IconsWidget
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

        self.geometry("480x380")
        self.wm_attributes('-type', 'splash')
        self.bind('<Escape>', lambda e: self.destroy())
        screen_width = self.winfo_screenwidth()
        self.attributes("-fullscreen", True)

        self.device_manager = SerialDeviceManager(config[DEVICES], [BUTTON])
        self.input_buttons = GPButtons(self.device_manager)

        self.canvas = tk.Canvas(self, bg='white')
        self.canvas.pack(fill=BOTH)

        self.pressed_button = None
        self.current_mode_index = 0

        #PAD
        #up
        self.canvas_buttons = {}
        up = [100, 90, 150, 20, 200, 90]
        self.canvas_buttons[UP] = self.canvas.create_polygon(up, fill='red')
        #canvas.tag_bind(up, '<ButtonPress-1>', self.on_object_click)

        #left
        points = [100, 90, 100, 180, 30, 135]
        self.canvas_buttons[LEFT] = self.canvas.create_polygon(points, fill='red')

        #down
        points = [100, 180, 150, 250, 200, 180]
        self.canvas_buttons[DOWN] = self.canvas.create_polygon(points, fill='red')

        #right
        points = [200, 90, 200, 180, 270, 135]
        self.canvas_buttons[RIGHT] = self.canvas.create_polygon(points, fill='red')

        self.canvas.create_rectangle(100, 90, 200, 180, fill="blue", outline='blue')
        self.mode = self.canvas.create_text(150, 145, text="HE", fill="black", font=('Helvetica 50 bold'))

        #BUTTONS
        #Y
        self.canvas_buttons[Y] = self.create_circle(420, 50, 35, self.canvas)
        #X
        self.canvas_buttons[X] = self.create_circle(320, 80, 35, self.canvas)
        #A
        self.canvas_buttons[A] = self.create_circle(420, 150, 35, self.canvas)
        #B
        self.canvas_buttons[B] = self.create_circle(320, 180, 35, self.canvas)

        self.feedback = self.canvas.create_text(210, 240, fill="black", font=('Helvetica 22 bold'))

        self.check_buttons()
        #self.check_external_buttons()

        self.send_messages()

    def send_messages(self):
        self.after(100, self.send_messages)
        if InputMessagesList().has_msg():
            ControlFactory().message.send(InputMessagesList().pop_msg())

    def create_circle(self, x, y, r, canvas): #center coordinates, radius
        x0 = x - r
        y0 = y - r
        x1 = x + r
        y1 = y + r
        return canvas.create_oval(x0, y0, x1, y1, fill="blue")

    def show_pressed_button(self):
        print("truc")

    def check_buttons(self):
        self.after(200, self.check_buttons)

        self.input_buttons.check_internal(SELECT_MODE[self.current_mode_index])
        self.input_buttons.check_external(SELECT_MODE[self.current_mode_index])

        messages = InputMessagesList().get_all()
        if messages:
            logging.info(messages)
            if self.actions_from_button(messages):
                InputMessagesList().add_msg(messages)
                self.canvas.itemconfigure(self.feedback, text=messages)
                self.after(3000, self.clean_feedback)
                Misc.exec_shell("DISPLAY=:0 xset s reset")

    def clean_feedback(self):
        self.canvas.itemconfigure(self.feedback, text="")

    def button_default(self):
        self.canvas.itemconfig(self.pressed_button, fill="blue")  # change color

    def actions_from_button(self, messages):
        if BL in messages and BR in messages:
            logging.fatal("restart app")
            os.execv(sys.executable, ['python'] + sys.argv)
        if SELECT in messages:
            self.current_mode_index = (self.current_mode_index + 1) % len(SELECT_MODE)
            logging.info("change mode index {} value {}".format(self.current_mode_index, SELECT_MODE[self.current_mode_index]))
            self.canvas.itemconfigure(self.mode, text=SELECT_MODE[self.current_mode_index])
            return False

        return True




    #def on_object_click(self):
