import logging
import tkinter as tk
from tkinter import TOP, X, BOTH
from tkinter.font import Font, BOLD

from dadou_utils.misc import Misc
from dadou_utils.utils_static import UtilsStatic

from dadoucontrol.control_static import ControlStatic

from dadoucontrol.control_factory import ControlFactory


class KeyboardFrame(tk.Frame):
    def __init__(self, parent):

        self.mod = 'A'
        self.deviceManager = ControlFactory().device_manager
        #self.serial_glove_left = self.deviceManager.gloveLeft
        #self.serial_glove_right = self.deviceManager.gloveRight

        tk.Frame.__init__(self, parent, bg='dark orange')
        self.pack(fill='both', expand=True, side='top')

        self.bold50 = Font(self.master, size=45, weight=BOLD)
        self.bold80 = Font(self.master, size=70, weight=BOLD)

        keys = [['1', '2', '3', 'A', 'E'],
                ['4', '5', '6', 'B', 'F'],
                ['7', '8', '9', 'C', 'G'],
                ['&', '0', '$', 'D', 'H']]

        grid = tk.Frame(self, bg='blue')
        grid.pack(fill='both', side='top', expand=True)

        for x in range(4):
            for y in range(5):
                self.create_cell(grid, x, y, keys[x][y])

        self.left_glove_feedback_panel = tk.Label(grid, bg="red", text='L', font=self.bold50, height=1, width=2)
        self.left_glove_feedback_panel.grid(row=0, column=6)

        self.right_glove_feedback_panel = tk.Label(grid, bg="red", text='R', font=self.bold50, height=1, width=2)
        self.right_glove_feedback_panel.grid(row=0, column=7)

        self.internet_label = tk.Label(grid, bg="green", text="I", font=self.bold50, width=2)
        self.internet_label.grid(row=0, column=8)

        self.right_panel_top = tk.Label(grid, bg="yellow", text='A', font=self.bold50, height=1, width=4)
        self.right_panel_top.grid(row=1, column=6, columnspan=3)

        self.right_panel_middle = tk.Label(grid, bg="turquoise", font=self.bold80, height=1, width=4)
        self.right_panel_middle.grid(row=2, column=6, columnspan=3, rowspan=2)

        self.check_internet()
        self.glove_input()
        self.check_plugged_device()

    def create_cell(self, grid, x, y, name):
        color = "blue"
        if ((x+y) % 2) == 0:
            color = "white"
        cell = tk.Button(grid, text=name, font=self.bold50, bg=color, command=lambda: self.click_button(name), activebackground='purple')
        cell.grid(row=x, column=y)

    def check_internet(self) -> None:
        self.after(500, self.check_internet)
        if Misc.is_connected():
            self.internet_label.config(bg="green")
        else:
            self.internet_label.config(bg="red")

    def click_button(self, name):
        if name in "ABCDEFGH":
            self.right_panel_top.config(text=name)
            self.mod = name
        else:
            self.right_panel_middle.config(text=self.mod+name)
            ControlFactory().message.send(self.mod+name, None)

    def check_plugged_device(self):
        self.after(500, self.check_plugged_device)
        if self.deviceManager.get_device(ControlStatic.GLOVE_LEFT):
            self.left_glove_feedback_panel.config(bg="green")
        else:
            self.left_glove_feedback_panel.config(bg="red")
        if self.deviceManager.get_device(ControlStatic.GLOVE_RIGHT):
            self.right_glove_feedback_panel.config(bg="green")
        else:
            self.right_glove_feedback_panel.config(bg="red")

    def glove_input(self) -> None:
        self.after(100, self.glove_input)
        devices = self.deviceManager.get_device_type(UtilsStatic.KEY_INPUT_KEY)
        for device in devices:
            msg = device.get_msg_separator()
            if msg:
                self.right_panel_middle.config(text=msg)
                ControlFactory().message.send(msg, None)


