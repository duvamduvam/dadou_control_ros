import logging
import os
import sys
from tkinter import BOTH, TOP, X, HORIZONTAL, LEFT, RIGHT

import tkinter as tk

from dadou_utils.misc import Misc
from dadou_utils.utils_static import BORDEAUX, FONT1, FONT2, ORANGE, PURPLE, YELLOW, CYAN

from dadoucontrol.control_config import config
from dadoucontrol.control_factory import ControlFactory


class SmallConfig(tk.Frame):


    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        #self.main = tk.Frame.__init__(self, parent, bg=PURPLE, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
        #self.main = tk.Frame.__init__(self, parent, bg=PURPLE, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
        self.exit_button = tk.Button(self, text='Exit', bg=config[BORDEAUX], width=10, font=config[FONT1],
                  command=exit)
        self.exit_button.grid(row=0, column=0)
        self.restart_button = tk.Button(self, text='Restart', width=10, bg=config[CYAN], font=config[FONT1],
                  command=self.restart)
        self.restart_button.grid(row=0, column=1)
        self.shutdown_button = tk.Button(self, text='Shutdown', width=10, bg=config[ORANGE], font=config[FONT1],
                  command=self.halt)
        self.shutdown_button.grid(row=1, column=0)
        self.reboot_button = tk.Button(self, text='Reboot', width=10, bg=config[PURPLE], font=config[FONT1],
                  command=self.reboot)
        self.reboot_button.grid(row=1, column=1)
        self.reload_usb_button = tk.Button(self, text='Reload USB', width=10, bg=config[PURPLE], font=config[FONT1],
                  command=self.reload_usb)
        self.reload_usb_button.grid(row=2, column=0)
        volume_scale = tk.Scale(self, from_=0, to=100, bg=config[YELLOW], tickinterval=10, length=240, orient=HORIZONTAL)
        volume_scale.grid(row=3, column=0)

        brightness_scale = tk.Scale(self, from_=0, to=1, resolution=0.05, bg=config[BORDEAUX], tickinterval=0.05, length=240, orient=HORIZONTAL)
        brightness_scale.grid(row=3, column=1)



        #volume_frame = tk.Frame(self)
        #volume_label = tk.Label(volume_frame, text="Volume")
        #volume_label.pack(fill=X, side=LEFT)
        #self.volume_scale = tk.Scale(volume_frame, from_=0, to=100, length=500, resolution=1, command=self.update_volume(), orient=HORIZONTAL)
        #self.volume_scale.pack(fill=X, side=LEFT)
        #volume_frame.pack(fill=X, side=TOP)

    #def update_volume(self):
    #    logging.info("volume changed {}".format(self.volume_scale.get()))

    def restart(self):
        os.execv(sys.executable, ['python'] + sys.argv)

    def halt(self):
        os.system("sudo shutdown -h")

    def reboot(self):
        os.system("sudo reboot")

    def reload_usb(self):
        ControlFactory().device_manager.update_devices()
    