import logging
import tkinter as tk
from tkinter.font import Font, BOLD

from dadoucontrol.com.serial_device_manager import SerialDeviceManager
from dadoucontrol.control_factory import ControlFactory


class KeyboardFrame(tk.Frame):
    def __init__(self, parent):

        tk.Frame.__init__(self, parent, bg='dark orange')
        self.pack(fill='both', expand=True, side='top')

        self.bold25 = Font(self.master, size=50, weight=BOLD)

        keys = [['1', '2', '3', 'A', 'E'],
                ['4', '5', '6', 'B', 'F'],
                ['7', '8', '9', 'C', 'G'],
                ['&', '0', '$', 'D', 'H']]

        grid = tk.Frame(self, bg='blue')
        grid.pack(fill='both', side='top', expand=True)

        for x in range(4):
            for y in range(5):
                self.create_cell(grid, x, y, keys[x][y])

        """first = tk.Label(grid, text='1', bg='white')
        first.grid(row=0, column=0, padx=100, sticky="nswe")
        first.grid_rowconfigure(0, minsize=100)

        second = tk.Label(grid, text='2', bg='blue')
        second.grid(row=0, column=1, padx=10, sticky="nswe")
        second.grid_rowconfigure(4, minsize=100)

        third = tk.Label(grid, text='3', bg='white')
        third.grid(row=0, column=2, padx=10, sticky="nswe")
        a = tk.Label(grid, text='A', bg='blue')
        a.grid(row=0, column=3, padx=10, sticky="nswe")

        four = tk.Label(grid, text='4', bg='white')
        four.grid(row=1, column=0, padx=10, sticky="nswe")
        five = tk.Label(grid, text='5', bg='blue')
        five.grid(row=1, column=1, padx=10, sticky="nswe")
        six = tk.Label(grid, text='6', bg='white')
        six.grid(row=1, column=2, padx=10, sticky="nswe")
        b = tk.Label(grid, text='B', bg='blue')
        b.grid(row=1, column=3, padx=10, sticky="nswe")"""

    def create_cell(self, grid, x, y, name):
        color = "blue"
        if ((x+y) % 2) == 0:
            color = "white"
        cell = tk.Button(grid, text=name, font=self.bold25, bg=color)
        cell.grid(row=x, column=y)
