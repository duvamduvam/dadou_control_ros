import logging
import tkinter as tk
from tkinter import TOP, X, BOTH
from tkinter.messagebox import showinfo, Message

from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.utils_static import BUTTON_GRID, INPUT_KEY, KEY, LORA, JOY, SLIDERS, CYAN, BORDEAUX, FONT1, YELLOW, \
    PURPLE, \
    GLOVE_LEFT, GLOVE_RIGHT, ORANGE, WHEELS, STOP, NAME, CMD, FONT2, MSG, DEVICE
from dadoucontrol.buttons.button_config import INPUT_KEYS, KEYS_MAPPING, BUTTONS_LAYOUT, Buttons

from dadoucontrol.control_config import config, RESTART_APP, SINGLE_GLOVE, SINGLE_GLOVE_9DOF
from dadoucontrol.control_factory import ControlFactory

class SmallControl(tk.Frame):
    def __init__(self, parent, mode, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent
        self.mode = mode

        self.pack(fill=BOTH, expand=True, side=TOP)

        self.grid_cell = []
        self.grid = tk.Frame(self, bg=config[CYAN])
        self.create_grid(BUTTONS_LAYOUT[self.mode])

        self.exec_input()

    def create_grid(self, items):
        for y in range(len(INPUT_KEYS)):
            row = []
            for x in range(len(INPUT_KEYS[y])):
                key = INPUT_KEYS[y][x]
                value = items[key][NAME]
                row.append(self.create_cell(grid=INPUT_KEYS, x=x, y=y, key=key, value=value))
            self.grid_cell.append(row)

        self.grid.pack(fill=BOTH, side=TOP, expand=True)

    def update_grid(self, items):
        for row in range(len(self.grid_cell)):
            for col in range(len(self.grid_cell[row])):
                but = self.grid_cell[row][col]
                key = INPUT_KEYS[row][col]
                value = items[key][NAME]
                but.configure(text=value)

    def create_cell(self, grid, x, y, key, value):
        color = config[CYAN]
        if ((x+y) % 2) == 0:
            color = config[YELLOW]
        cell = tk.Button(self.grid, text=value, font=config[FONT2], bg=color, command=lambda: self.click_button(text=value, key=key), activebackground=config[PURPLE], height=2, width=11)
        cell.grid(row=x, column=y)
        return cell

    def click_button(self, text, key):
        value = BUTTONS_LAYOUT[self.mode][key][CMD]
        logging.info("button \"{}\" => \"{}\"".format(text, value))
        ControlFactory().message.send_multi_ws(value)

    def exec_input(self):
        self.after(100, self.exec_input)

        if InputMessagesList().has_msg():
            msg = InputMessagesList().pop_msg()

            if "glove" in msg[DEVICE]:
                value = Buttons.get(self.mode, msg[MSG])#BUTTONS_LAYOUT[self.mode][KEYS_MAPPING[msg[MSG]]][CMD]
                if not value:
                    return

                logging.info("input msg {}".format(value))
                key_list = list(value.keys())
                self.parent.show_popup("{} : {}".format(key_list[0], value[key_list[0]]))
                ControlFactory().message.send_multi_ws(value)







