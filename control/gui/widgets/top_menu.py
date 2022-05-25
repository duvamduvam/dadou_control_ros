import tkinter as tk
from tkinter import Button
from tkinter import E, W, N, S

class TopMenu(tk.Frame):

    GLOVE_TXT = 'Glove'
    SEQUENCES_TXT = 'Sequences'
    CONFIG_TXT = 'Config'

    def __init__(self, parent, main, *args, **kwargs):
        tk.Frame.__init__(self, parent)
        self.grid(row=0, column=0, sticky=W + E)
        btn_Glove = Button(self, text=self.GLOVE_TXT, command=lambda: self.show_frame("GloveFrame"))
        btn_Glove.grid(row=0, column=0, padx=(10), pady=10)
        btn_Sequence = Button(self, text=self.SEQUENCES_TXT)
        btn_Sequence.grid(row=0, column=1, padx=(10), pady=10)
        btn_Config = Button(self, text=self.CONFIG_TXT)
        btn_Config.grid(row=0, column=2, padx=(10), pady=10)