import logging
import tkinter as tk
from tkinter import TOP, X, BOTH
from tkinter.font import Font, BOLD

from dadou_utils.misc import Misc
from dadou_utils.utils_static import INPUT_KEY, KEY, LORA, JOY, SLIDERS
from dadoucontrol.control_static import GLOVE_LEFT, GLOVE_RIGHT, ORANGE, BORDEAUX, YELLOW, CYAN, PURPLE, FONT1
from dadoucontrol.control_factory import ControlFactory


class KeyboardFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.mod = 'A'
        self.deviceManager = ControlFactory().device_manager
        #self.serial_glove_left = self.deviceManager.gloveLeft
        #self.serial_glove_right = self.deviceManager.gloveRight

        self.pack(fill=BOTH, expand=True, side=TOP)

        FONT1 = Font(self.master, size=45, weight=BOLD)
        self.bold80 = Font(self.master, size=70, weight=BOLD)

        keys = [['1', '2', '3', 'A', 'E'],
                ['4', '5', '6', 'B', 'F'],
                ['7', '8', '9', 'C', 'G'],
                ['&', '0', '$', 'D', 'H']]

        grid = tk.Frame(self, bg=CYAN)
        grid.pack(fill=BOTH, side=TOP, expand=True)

        for x in range(4):
            for y in range(5):
                self.create_cell(grid, x, y, keys[x][y])

        self.lora_feedback_panel = tk.Label(grid, bg=BORDEAUX, text='Lo', font=FONT1, height=1, width=2)
        self.lora_feedback_panel.grid(row=0, column=6)

        self.wifi_feedback_panel = tk.Label(grid, bg=BORDEAUX, text='Wi', font=FONT1, height=1, width=2)
        self.wifi_feedback_panel.grid(row=0, column=7)

        self.internet_label = tk.Label(grid, bg=ORANGE, text="I", font=FONT1, width=2)
        self.internet_label.grid(row=0, column=8)

        self.left_glove_feedback_panel = tk.Label(grid, bg=BORDEAUX, text='L', font=FONT1, height=1, width=2)
        self.left_glove_feedback_panel.grid(row=1, column=6)

        self.right_glove_feedback_panel = tk.Label(grid, bg=BORDEAUX, text='R', font=FONT1, height=1, width=2)
        self.right_glove_feedback_panel.grid(row=1, column=7)

        self.joy_feedback_panel = tk.Label(grid, bg=BORDEAUX, text='J', font=FONT1, height=1, width=2)
        self.joy_feedback_panel.grid(row=1, column=8)

        self.sliders_feedback_panel = tk.Label(grid, bg=BORDEAUX, text='S', font=FONT1, height=1, width=2)
        self.sliders_feedback_panel.grid(row=2, column=6)

        self.right_panel_top = tk.Label(grid, bg=YELLOW, text='A', font=FONT1, height=1, width=4)
        self.right_panel_top.grid(row=2, column=7, columnspan=2)

        self.right_panel_middle = tk.Label(grid, bg=CYAN, font=FONT1, height=1, width=4)
        self.right_panel_middle.grid(row=3, column=6, columnspan=3, rowspan=2)

        self.check_internet()
        self.check_glove_input()
        self.check_plugged_device()
        self.check_joystick_input()
        self.check_sliders_input()

    def create_cell(self, grid, x, y, name):
        color = CYAN
        if ((x+y) % 2) == 0:
            color = YELLOW
        cell = tk.Button(grid, text=name, font=FONT1, bg=color, command=lambda: self.click_button(name), activebackground=PURPLE)
        cell.grid(row=x, column=y)

    def check_internet(self) -> None:
        self.after(500, self.check_internet)
        #TODO fix slow startup
        """if Misc.is_connected():
            self.internet_label.config(bg="green")
        else:
            self.internet_label.config(bg=BORDEAUX)"""

    def click_button(self, key):
        if key in "ABCDEFGH":
            self.right_panel_top.config(text=key)
            self.mod = key
        else:
            self.right_panel_middle.config(text=self.mod + key)
            ControlFactory().message.send({KEY:self.mod + key})

    def check_plugged_device(self):
        self.after(500, self.check_plugged_device)
        self.update_feedback_panel(self.left_glove_feedback_panel, self.deviceManager.get_device(GLOVE_LEFT))
        self.update_feedback_panel(self.right_glove_feedback_panel, self.deviceManager.get_device(GLOVE_RIGHT))
        self.update_feedback_panel(self.lora_feedback_panel, self.deviceManager.get_device(LORA))
        self.update_feedback_panel(self.joy_feedback_panel, self.deviceManager.get_device(JOY))
        self.update_feedback_panel(self.sliders_feedback_panel, self.deviceManager.get_device(SLIDERS))

    def update_feedback_panel(self, label:tk.Label, activ:bool):
        if activ:
            label.config(bg=YELLOW)
        else:
            label.config(bg=BORDEAUX)

    def check_glove_input(self):
        self.after(100, self.check_glove_input)
        devices = self.deviceManager.get_device_type(INPUT_KEY)
        for device in devices:
            msg = device.get_msg_separator()
            if msg:
                self.right_panel_middle.config(text=msg)
                ControlFactory().message.send({KEY: msg})

    def check_joystick_input(self):
        self.after(100, self.check_joystick_input)
        joy = self.deviceManager.get_device(JOY)
        if joy:
            msg = joy.get_msg_separator()
            if msg:
                self.right_panel_middle.config(text=msg)
                ControlFactory().message.send({JOY: msg})

    def check_sliders_input(self):
        self.after(100, self.check_sliders_input)
        sliders = self.deviceManager.get_device(SLIDERS)
        if sliders:
            msg = sliders.get_msg_separator()
            if msg:
                self.right_panel_middle.config(text=msg)
                ControlFactory().message.send_sliders(msg)

