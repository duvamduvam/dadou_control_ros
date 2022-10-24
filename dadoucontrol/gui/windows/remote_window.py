import logging
import tkinter as tk

from dadou_utils.misc import Misc


class RemoteFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill='both', expand=True, side='top')

        left_menu = tk.Frame(self, width=50, bg='blue')
        left_menu.pack(fill='y', ipadx=20, side='left')

        tk.Button(left_menu, text='streaming', command=self.start_streaming).grid(row=3, column=6, padx=10)

    def start_streaming(self):
        Misc.exec_shell('ffplay tcp://192.168.1.127:8888 -vf "setpts=N/30" -fflags nobuffer -flags low_delay -framedrop & ')
