import tkinter as tk
from tkinter import Label
from tkinter import LabelFrame
from tkinter import E, W, N, S

from control.com.serial_device import SerialDevice
from control.com.serial_device_manager import SerialDeviceManager


class SequenceFrame(tk.Frame):

    def __init__(self, parent):
        tk.Frame.__init__(self, parent, bg='grey')
        self.pack(fill='both', expand=True, side='top')

        left_menu = tk.Frame(self, bg='blue')
        left_menu.pack(fill='y', ipadx=100, side='left')

        label1 = tk.Label(left_menu, bg='orange')
        label1.pack(ipady=50, fill='x', side='top')

        label2 = tk.Label(left_menu, bg='green')
        label2.pack(ipady=50, fill='x', side='top')

        center = tk.Frame(self, bg='black')
        center.pack(fill='both', expand=True, side='left')

        layer1 = tk.Label(center, bg='#4a7abc')
        layer1.pack(ipady=50, fill='x', side='top')

        layer2 = tk.Label(center, bg='#7c388f')
        layer2.pack(ipady=50, fill='x', side='top')

        layer3 = tk.Label(center, bg='#8f7b38')
        layer3.pack(ipady=50, fill='x', side='top')




