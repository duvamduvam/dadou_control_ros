import tkinter as tk

from dadoucontrol.gui.windows.sequences.face_frame import FaceFrame
from dadoucontrol.gui.windows.sequences.music_frame import MusicFrame
from dadoucontrol.gui.windows.sequences.navigation_widget import NavigationWidget
from dadoucontrol.gui.windows.sequences.neck_frame import NeckFrame
from dadoucontrol.gui.windows.sequences.wheels_frame import WheelsFrame

from dadoucontrol.gui.windows.sequences.sequences_manager_widget import SequencesManagerWidget


class SequenceFrame(tk.Frame):

    current_position = 0

    def __init__(self, parent):
        tk.Frame.__init__(self, parent, bg='grey')
        self.pack(fill='both', expand=True, side='top')

        self.left_menu = tk.Frame(self, bg='blue', width=100)
        self.left_menu.pack(fill='y', side='left')

        music_frame = MusicFrame(self, 'red')

        NavigationWidget(self.left_menu, width=100)
        SequencesManagerWidget(self.left_menu, music_frame)


        face_frame = FaceFrame(self, 'face', 'green')
        lights_frame = FaceFrame(self, 'lights', 'yellow')
        neck_frame = NeckFrame(self, 'orange')
        wheels_frame = WheelsFrame(self, 'violet')

    def open_popup(self, parent):
        top = tk.Toplevel(parent)
        top.geometry("500x250")
        top.title("Child Window")
        tk.Label(top, text="Hello World!", font=('Mistral 18 bold')).place(x=150, y=80)
