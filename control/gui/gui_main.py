from tkinter import Button, Frame
import tkinter as tk

from control.gui.widgets.glove_frame import GloveFrame
from control.gui.widgets.sequence_frame import SequenceFrame
from control.gui.widgets.top_menu import TopMenu
from tkinter import E, W, N, S


class MainGui(tk.Tk):

    GLOVE_TXT = 'Glove'
    SEQUENCES_TXT = 'Sequences'
    CONFIG_TXT = 'Config'

    mainFrame = None

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.top_menu = tk.LabelFrame(container)
        self.top_menu.pack(side="top", fill="x", pady=10)
        btn_Glove = Button(self.top_menu, text=self.GLOVE_TXT, command=lambda: self.show_frame("GloveFrame"))
        btn_Glove.pack(side="top", fill="x", pady=10)
        btn_Sequence = Button(self.top_menu, text=self.SEQUENCES_TXT)
        btn_Config = Button(self.top_menu, text=self.CONFIG_TXT)


        #buttons_frame = TopMenu(self.master, self)
        #self.centerFrame = GloveFrame(self.master, text="Text Box", padx=5, pady=5)
        #centerFrame = App(self.master)

        #App(master)

        self.frames = {}
        for F in (GloveFrame, SequenceFrame):
            page_name = F.__name__
            mainFrame = F(parent=container, controller=self)
            self.frames[page_name] = mainFrame

            # put all of the pages in the same location;
            # the one on the top of the stacking order
            # will be the one that is visible.
        #    frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("GloveFrame")

    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()


