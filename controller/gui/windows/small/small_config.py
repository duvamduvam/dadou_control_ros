import logging
import os
import sys
from tkinter import BOTH, TOP, X, HORIZONTAL, LEFT, RIGHT

import tkinter as tk

from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.misc import Misc
from dadou_utils.utils_static import BORDEAUX, FONT1, FONT22, ORANGE, PURPLE, YELLOW, CYAN, CONFIG, SPEED, BRIGHTNESS, \
    WHEELS, ROBOT_LIGHTS, RANDOM

from controller.control_config import config
from controller.control_factory import ControlFactory


class SmallConfig(tk.Frame):

    def __init__(self, parent, node, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)

        self.node = node

        self.pack(fill=BOTH, expand=True, side=TOP)

        #self.buttons = tk.Frame(self)
        #self.buttons.pack(fill=BOTH, expand=True, side=TOP)

        #self.buttons.grid_columnconfigure(0, weight=1)  # Configure la colonne 0 pour qu'elle prenne l'espace supplémentaire
        #self.buttons.grid_columnconfigure(1, weight=1)  # Configure la colonne 1 de la même manière
        #self.buttons.grid_columnconfigure(2, weight=1)  # Et la colonne 2

        self.grid_columnconfigure(0, weight=1)  # Configure la colonne 0 pour qu'elle prenne l'espace supplémentaire
        self.grid_columnconfigure(1, weight=1)  # Configure la colonne 1 de la même manière
        self.grid_columnconfigure(2, weight=1)  # Et la colonne 2

        self.exit_button = tk.Button(self, text='Exit', bg=config[BORDEAUX], font=config[FONT22],
                  command=exit)
        self.exit_button.grid(row=0, column=0, sticky="ew")
        self.restart_button = tk.Button(self, text='Reload', bg=config[CYAN], font=config[FONT22],
                  command=self.restart)
        self.restart_button.grid(row=0, column=1, sticky="ew")
        self.shutdown_button = tk.Button(self, text='Shutdown', bg=config[ORANGE], font=config[FONT22],
                  command=self.halt)
        self.shutdown_button.grid(row=0, column=2, sticky="ew")
        self.reboot_button = tk.Button(self, text='Reboot', bg=config[PURPLE], font=config[FONT22],
                  command=self.reboot)
        self.reboot_button.grid(row=1, column=0, sticky="we")
        self.random_value = tk.IntVar()
        self.random_button = tk.Checkbutton(self, text='Random', variable=self.random_value, onvalue=1, offvalue=0, bg=config[PURPLE], font=config[FONT22],
                  command=self.send_random_conf)
        self.random_button.grid(row=1, column=1, sticky="we")

        self.reload_usb_button = tk.Button(self, text='Reload USB', width=10, bg=config[BORDEAUX], font=config[FONT22],
                  command=self.reload_usb)
        self.reload_usb_button.grid(row=1, column=2, sticky="we")

        #self.sliders = tk.Frame(self)
        #self.sliders.pack(fill=BOTH, expand=True, side=TOP)

        #self.sliders.grid_columnconfigure(0, weight=1)  # Configure la colonne 0 pour qu'elle prenne l'espace supplémentaire

        self.speed_scale = tk.Scale(self, from_=20, to=100, bg=config[YELLOW], resolution=20, tickinterval=20, length=240, orient=HORIZONTAL, command=self.send_speed_conf)
        self.speed_scale.grid(row=2, column=0, sticky="ew")

        self.brightness_scale = tk.Scale(self, from_=0.05, to=0.3, resolution=0.05, bg=config[BORDEAUX], tickinterval=0.05, length=240, orient=HORIZONTAL, command=self.send_brightness_conf)
        self.brightness_scale.grid(row=2, column=1, sticky="ew")

    def send_speed_conf(self, speed):
        self.node.publish({SPEED: speed})

    def send_brightness_conf(self, brightness):
        self.node.publish({BRIGHTNESS: brightness})

    def send_random_conf(self):
        self.node.publish({RANDOM: bool(self.random_value.get())})

    def restart(self):
        os.execv(sys.executable, ['python'] + sys.argv)

    def halt(self):
        os.system("sudo shutdown -h")

    def reboot(self):
        os.system("sudo reboot")

    def reload_usb(self):
        ControlFactory().devices_manager.update_devices()
    