from tkinter import BOTH, TOP, X

import tkinter as tk

from control_static import PURPLE, BORDEAUX, FONT1


class ConfigFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.main = tk.Frame.__init__(self, parent, bg=PURPLE, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
        self.exit_button = tk.Button(self.main, text='Exit', bg=BORDEAUX, font=FONT1,
                  command=exit)
        self.exit_button.pack(fill=X, side=TOP)

    def exit(self):
        exit()