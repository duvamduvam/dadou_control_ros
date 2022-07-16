import tkinter as tk                # dadoutils 3
from tkinter import font as tkfont  # dadoutils 3

from dadoucontrol.gui.windows.expression_window import ExpressionFrame
from dadoucontrol.gui.windows.glove_window import GloveFrame
from dadoucontrol.gui.windows.sequence_window import SequenceFrame

#https://www.hashbangcode.com/article/using-events-tkinter-canvas-elements-python

class MainGui(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.EXPRESSION_FRAME = 'EXPRESSION'
        self.GLOVE_FRAME = 'GLOVE'
        self.SEQUENCE_FRAME = 'SEQUENCE'
        self.CONFIG_FRAME = 'CONFIG'

        self.title_font = tkfont.Font(family='Helvetica', size=18, weight="bold", slant="italic")

        self.geometry("1600x1024")

        menu = tk.Frame(self, bg='orange')
        menu.pack(fill='x', side='top')

        tk.Button(menu, text='Glove', bg='red', command=lambda: self.show_frame(self.GLOVE_FRAME)).pack(ipadx=10, ipady=10, fill='x', expand=True, side='left')
        tk.Button(menu, text='Sequence', bg='blue', command=lambda: self.show_frame(self.SEQUENCE_FRAME)).pack(ipadx=10, ipady=10, fill='x', expand=True, side='left')
        tk.Button(menu, text='Expression', bg='grey', command=lambda: self.show_frame(self.EXPRESSION_FRAME)).pack(ipadx=10, ipady=10, fill='x', expand=True, side='left')
        tk.Button(menu, text='Config', bg='green', command=lambda: self.show_frame(self.CONFIG_FRAME)).pack(ipadx=10, ipady=10, fill='x', expand=True, side='left')


        self.main = tk.Frame(self, bg='yellow')
        self.main.pack(fill='both', expand=True, side='top')

    def show_frame(self, frame_name):
        self.main.destroy()
        if frame_name == self.GLOVE_FRAME:
            self.main = GloveFrame(self)
        elif frame_name == self.SEQUENCE_FRAME:
            self.main = SequenceFrame(self)
        elif frame_name == self.EXPRESSION_FRAME:
            self.main = ExpressionFrame(self)
        elif frame_name == self.CONFIG_FRAME:
            self.main = Config(self)

class Config(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent, bg='violet')
        self.pack(fill='both', expand=True, side='top')
